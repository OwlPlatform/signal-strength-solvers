/*
 * Copyright (c) 2012 Bernhard Firner and Rutgers University
 * All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301, USA
 * or visit http://www.gnu.org/licenses/gpl-2.0.html
 */

/*******************************************************************************
 * @file mobility_solver.cpp
 * Use variance information from the world model to provide mobility information
 * to the world model.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <algorithm>
#include <atomic>
#include <cmath>
#include <deque>
#include <functional>
#include <map>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>
#include <utility>
#include <vector>

//TODO Remove this (used for usleep) and replace with C++0x sleep mechanism
#include <unistd.h>

#include <owl/client_world_connection.hpp>
#include <owl/grail_types.hpp>
#include <owl/netbuffer.hpp>
#include <owl/solver_world_connection.hpp>
#include <owl/world_model_protocol.hpp>

using std::deque;
using std::map;
using std::pair;
using std::tuple;
using std::u16string;
using std::vector;

using world_model::Attribute;
using world_model::grail_time;
using world_model::URI;

typedef pair<unsigned char, TransmitterID> UniqueTxer;

std::u16string txerToUString(UniqueTxer& ut) {
  std::string str = std::to_string(ut.first) + "." + std::to_string(ut.second.lower);
  return u16string(str.begin(), str.end());
}

/**
 * Handler for new sensor IDs from the world model
 * Add new sensor IDs to the transmitter_to_uri map to store so that
 * sensor mobility can be associated with object mobility.
 */
void sensorIDHandler(StepResponse& sr, map<u16string, URI>& transmitter_to_uri, std::mutex& map_mutex) {
  while (sr.hasNext()) {
    //Get world model updates
    world_model::WorldState ws = sr.next();

    //Lock the map mutex before adding values to the transmitter_to_uri map
    std::unique_lock<std::mutex> lck(map_mutex);

    //Check each object to match sensor IDs to world model object IDs
    for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
      const URI& uri = I.first;
      //Object may have multiple sensors so scan each one
      for (const Attribute& attr : I.second) {
        //Transmitters are stored as one byte of physical layer and 16 bytes of ID
        grail_types::transmitter txid = grail_types::readTransmitter(attr.data);
        std::string tx_name = to_string(txid.phy) + "." + to_string(txid.id.lower);
        if (transmitter_to_uri.find(u16string(tx_name.begin(), tx_name.end())) == transmitter_to_uri.end()) {
          std::cout<<"Adding new transmitter to URI: "<<tx_name<<"->"<<std::string(uri.begin(), uri.end())<<'\n';
        }
        transmitter_to_uri[u16string(tx_name.begin(), tx_name.end())] = uri;
      }
    }
  }
}

/**
 * Handler for sensor variance data.
 * When the variance of a sensor's RSS values go above a threshold
 * they are probably moving.
 * Sensors are associated to objects by data placed in the
 * transmitter_to_uri map by the sensorIDHandler.
 */
void varianceHandler(SolverWorldModel& swm, StepResponse& sr, map<u16string, URI>& transmitter_to_uri, std::mutex& map_mutex, double threshold) {
  //Keep track of the current status we've reported
  map<URI, bool> moving;

  while (sr.hasNext()) {
    //Get world model updates
    world_model::WorldState ws = sr.next();
    //Make a list of new mobility solutions
    vector<SolverWorldModel::AttrUpdate> solns;

    {
      //Lock the map mutex before reading values from the transmitter_to_uri map
      std::unique_lock<std::mutex> lck(map_mutex);

      for (const std::pair<URI, std::vector<Attribute>>& I : ws) {
        //Process if there is a variance attribute
        if (not I.second.empty()) {
          const URI& tx_uri = I.first;
          //See if this transmitter is known
          if (transmitter_to_uri.end() != transmitter_to_uri.find(tx_uri)) {
            //Translate the transmitter ID into an object's name
            URI& uri = transmitter_to_uri[tx_uri];
            const Attribute& attr = I.second.at(0);
            double variance = readPrimitive<double>(attr.data, 0);
            //Threshold is for standard deviation.
            bool is_moving = sqrt(variance) >= threshold;
            if (moving.find(uri) == moving.end() or is_moving != moving[uri]) {
              //Make the transition elastic
              if ((not moving[uri] and sqrt(variance) >= 1.05*threshold) or
                  (moving[uri] and sqrt(variance) <= 0.95 * threshold)) {
                moving[uri] = is_moving;
                SolverWorldModel::AttrUpdate soln{u"mobility", attr.creation_date, uri, vector<uint8_t>()};
                pushBackVal<uint8_t>(is_moving ? 1 : 0, soln.data);
                std::cerr<<"Sending mobility solution for "<<std::string(uri.begin(), uri.end())<<": "<<(is_moving ? "moving\n" : "not moving\n");
                solns.push_back(soln);
              }
            }
          }
        }
      }
    }
    if (not solns.empty()) {
      //Do not create URIs for these entries, just send the data
      swm.sendData(solns, false);
    }
  }
}

int main(int ac, char** av) {

  if (ac == 2 and std::string(av[1]) == "-?") {
    std::cout<< "name: RSS Mobility Detection";
    std::cout<< "arguments: worldmodel wm_solver wm_client";
    std::cout<< "description: Creates mobility information from signal strength values.";
    std::cout<< "provides: mobility";
    std::cout<< "requires: sensor\\..*";
    std::cout<< "requires: average variance";
    return 0;
  }

  if (4 != ac) {
    std::cerr<<"This program needs 3 arguments:\n";
    std::cerr<<"\t"<<av[0]<<" <world model ip> <solver port> <client port>\n";
    std::cerr<<"The solution type \"average variance\" will be used for mobility detection.\n";
    return 0;
  }

  //World model IP and ports
  std::string wm_ip(av[1]);
  int solver_port = std::stoi(std::string((av[2])));
  int client_port = std::stoi(std::string((av[3])));

  //Variance threshold for mobility detection
  double threshold = 3.68;

  //Set up the solver world model connection;
  std::string origin = "mobility_solver (threshold "+std::to_string(threshold)+")";
  //Provide mobility as a non-transient type
  std::vector<std::pair<u16string, bool>> type_pairs{{u"mobility", false}};
  SolverWorldModel swm(wm_ip, solver_port, type_pairs, u16string(origin.begin(), origin.end()));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model as a solver - aborting.\n";
    return 0;
  }
  
  ClientWorldConnection cwc(wm_ip, client_port);
  if (not cwc.connected()) {
    std::cerr<<"Could not connect to the world model as a client - aborting.\n";
    return 0;
  }

  //Get attributes named "average variance"
  //If that parameter goes above threshold declare that item moving.
  URI any_sensor = u".*";
  vector<URI> sensor_attributes{u"sensor.*"};
  //Get updates once every second
  grail_time interval = 1000;
  StepResponse sensor_response = cwc.streamRequest(any_sensor, sensor_attributes, interval);
  //cwm.setupSynchronousDataStream(any_sensor, sensor_attributes, interval, (uint32_t)Ticket::sensors);
  URI any_link = u".*";
  vector<URI> link_attributes{u"average variance"};
  StepResponse variance_response = cwc.streamRequest(any_link, link_attributes, interval);
  //cwm.setupSynchronousDataStream(any_link, link_attributes, interval, (uint32_t)Ticket::variance);

  //Keep track of transmitter IDs to object IDs in the world model
  map<u16string, URI> transmitter_to_uri;
  //Protect access to the transmitter_to_uri map so that we can use two threads.
  std::mutex map_mutex;

  //Send sensor data to the sensorIDHandler
  std::thread sensor_id_thread(sensorIDHandler, std::ref(sensor_response),
      std::ref(transmitter_to_uri), std::ref(map_mutex));

  //Handle variance in this thread
  varianceHandler(swm, variance_response, transmitter_to_uri, map_mutex, threshold);

  return 0;
}

