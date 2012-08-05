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
 * @file localization_evaluator.cpp
 * Replay trace data from a provided file. Signal statistics will be sent to
 * a running world model and the location results provided by any running
 * localization programs will be evaluated compared to the ground truths of the
 * data.
 * This program will create its own entries for the locations of receivers and
 * training points as on-demand values, but a solver might be influenced by
 * any existing values in the world model so a clean world model should be used
 * for evaluation purposes. This allows users to intentionally place values
 * into the world model to add more information for localization solvers.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <algorithm>
#include <cmath>
#include <deque>
#include <fstream>
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

#include <owl/grail_types.hpp>
#include <owl/solver_aggregator_connection.hpp>
#include <owl/solver_world_connection.hpp>
#include <owl/netbuffer.hpp>
#include <owl/sample_data.hpp>
#include <owl/world_model_protocol.hpp>

using std::deque;
using std::get;
using std::map;
using std::pair;
using std::string;
using std::tuple;
using std::u16string;
using std::vector;

using world_model::grail_time;

///Definitions to describe radio links.
typedef tuple<unsigned char, TransmitterID, ReceiverID> Link;
typedef pair<unsigned char, TransmitterID> UniqueTxer;
typedef pair<Timestamp, float> Signal;

///Convert a link to a ID for the world model
u16string linkToUString(Link& link) {
  string str = std::to_string(get<0>(link)) + "." +
    std::to_string(get<1>(link).lower) + "." + std::to_string(get<2>(link).lower);
  return u16string(str.begin(), str.end());
}

///Convert a transmitter to an ID of the form <physical layer>.<transmitter id>
u16string txerToUString(UniqueTxer& ut) {
  string str = std::to_string(ut.first) + "." + std::to_string(ut.second.lower);
  return u16string(str.begin(), str.end());
}


///Store a link's RSS in the signal map
void storeLinkRSS(SampleData& sample, map<Link, deque<Signal>>& signals, std::mutex& m) {
  if (sample.valid) {
    //Lock the mutex to gain control of the signals map
    std::unique_lock<std::mutex> lck(m);
    //Add this sample into the signals map
    //TODO FIXME The rx timestamp is not synchronized with this computer
    // Using the current local time
    //signals[Link{sample.physical_layer, sample.tx_id, sample.rx_id}].push_back(std::make_pair(sample.rx_timestamp, sample.rss));
    signals[Link{sample.physical_layer, sample.tx_id, sample.rx_id}].push_back(std::make_pair(world_model::getGRAILTime(), sample.rss));
  }
}

///Get the average of the doubles in a vector
double getAverage(vector<double> v) {
  return std::accumulate(v.begin(), v.end(), 0.0, [&](double a, double b){return a + (b/v.size());});
}

vector<string> split(string str, char c = ' ') {
  return accumulate(str.begin(), str.end(), vector<string>{""},
      [&](vector<string> v, char cc) {
      if (c == cc) { v.push_back(""); return v;}
      v.back().push_back(cc);
      return v;});
}

int main(int ac, char** av) {
  if (ac == 2 and string(av[1]) == "-?") {
    std::cout<< "name: Localization Evaluation Solver\n";
    std::cout<< "arguments: worldmodel wm_solver wm_client training_file trace_file\n";
    std::cout<< "description: Tests a running localization solver.\n";
    std::cout<< "provides: link rss\n";
    std::cout<< "provides: link variance\n";
    std::cout<< "provides: average variance\n";
    std::cout<< "provides: link average\n";
    std::cout<< "provides: link median\n";
    std::cout<< "provides: receivers.vector<sized string>\n";
    std::cout<< "provides: fingerprint.vector<RSS>\n";
    return 0;
  }

  if (6 != ac) {
    std::cerr<<"This program needs 5 arguments:\n";
    std::cerr<<"\t"<<av[0]<<" <world model ip> <solver port> <client port> training_file trace_file\n";
    std::cerr<<"The training file has region configuration, receiver locations, and training points.\n";
    std::cerr<<"The very first line of the rxlocation file should\n";
    std::cerr<<"have the maximum x, y, and possibly z values for the region.\n";
    std::cerr<<"The z offset is optional, but all receiver and training point locations must have\n";
    std::cerr<<"the same number of coordinates as the region.\n";
    std::cerr<<'\n';
    std::cerr<<"A receiver location appears in a single line with the format 'rxid xoffset yoffset (zoffset)'.\n";
    std::cerr<<"After all receivers are declared any number of training points can appear\n";
    std::cerr<<"with the format 'xoffset yoffset (zoffset) {rxid rss}+'.\n";
    std::cerr<<"Training points will be sent to the world model with two.\n";
    std::cerr<<"vector attributes containing receiver names and their RSS values.\n";
    return 0;
  }

  //World model IP and port
  string wm_ip(av[1]);
  int wm_solver = std::stoi(string((av[2])));
  int wm_client = std::stoi(string((av[3])));

  //Set up the solver world model connection;
  string origin = "localization_evaluator";
  //Provide signal statistics and training data as on demand types.
  //Link variance is between a transmitter and a receiver.
  //Average variance is the average of all link variances for a transmitter
  std::vector<std::pair<u16string, bool>> type_pairs{{u"link variance", true},
                                                     {u"average variance", true},
                                                     {u"link average", true},
                                                     {u"link median", true},
                                                     {u"link rss", true},
                                                     {u"location.xoffset", true},
                                                     {u"location.yoffset", true},
                                                     {u"location.zoffset", true},
                                                     {u"location.maxx", true},
                                                     {u"location.maxy", true},
                                                     {u"location.maxz", true}};
  SolverWorldModel swm(wm_ip, wm_solver, type_pairs, u16string(origin.begin(), origin.end()));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model as a solver - aborting.\n";
    return 0;
  }

  //Now read through the training file and send all of the training data.
  //Open the rxlocation file and trace files
  std::ifstream train_in(av[4]);
  if (not train_in) {
    std::cerr<<"Error opening training_file '"<<av[4]<<"'\n";
    return 0;
  }
  std::ifstream trace_in(av[5]);
  if (not trace_in) {
    std::cerr<<"Error opening trace_file '"<<av[5]<<"'\n";
    return 0;
  }

  //Iterate through the file line by line, but first get the dimensions
  string line;
  int num_dimensions = 0;

  //Maximum values for the region
  std::vector<double> max_values;


  if (std::getline(train_in, line)) {
    std::vector<SolverWorldModel::AttrUpdate> region_attrs;
    std::vector<string> parts = split(line);
    std::vector<u16string> attr_names{u"maxx", u"maxy", u"maxz"};
    if (3 < parts.size()) {
      std::cerr<<"First line of training file specifies more than three dimensions for the region! Aborting!\n";
      return 0;
    }
    for (size_t i = 0; i < parts.size(); ++i) {
      SolverWorldModel::AttrUpdate max_soln{attr_names[i], 0, u"evaluation_region", vector<uint8_t>()};
      pushBackVal(std::stod(parts[i]), max_soln.data);
      region_attrs.push_back(max_soln);
      //Save the maximum values to double-check the training point and landmark locations
      max_values.push_back(std::stod(parts[i]));
    }
    num_dimensions = parts.size();
    std::cout<<"Using "<<num_dimensions<<" dimensions for localization region.\n";
    swm.sendData(region_attrs, false);
  }
  else {
    std::cerr<<"Problem with training file: no region dimensions.\n";
    return 0;
  }

  //Get receiver locations
  size_t rxlocations = 0;
  while (std::getline(train_in, line)) {
    std::vector<SolverWorldModel::AttrUpdate> rx_attrs;
    std::vector<string> parts = split(line);
    std::vector<u16string> attr_names{u"location.xoffset", u"location.yoffset", u"location.zoffset"};
    if (num_dimensions+1 != parts.size()) {
      if (num_dimensions+2 <= parts.size()) {
        //Switching to transmitters
        break;
      }
      else {
        std::cerr<<"Receiver location does not match the given number of dimensions (line is "+line+")\n";
        return 0;
      }
    }
    //Increment the number of receiver locations
    ++rxlocations;
    //The receiver name will be based on its ID. (convert from char8 to char16)
    u16string rx_name = u"evaluation_region.landmark." + u16string(parts[0].begin(), parts[0].end());
    //Store the receiver's sensor ID as an attribute
    {
      SolverWorldModel::AttrUpdate rxsensor{u"sensor.receiver", 0, rx_name, vector<uint8_t>()};
      uint128_t rxid = std::stoll(parts[0]);
      grail_types::writeTransmitter(grail_types::transmitter{1, rxid}, rxsensor.data);
      rx_attrs.push_back(rxsensor);
    }
    //Now get the x, y, and possibly z locations
    for (size_t i = 0; i < num_dimensions; ++i) {
      SolverWorldModel::AttrUpdate offset{attr_names[i], 0, rx_name, vector<uint8_t>()};
      pushBackVal(std::stod(parts[i+1]), offset.data);
      rx_attrs.push_back(offset);
    }
    //Send this receiver's attributes to the world model
    swm.sendData(rx_attrs, false);
  }
  std::cerr<<"Using "<<rxlocations<<" receiver locations.\n";

  //Get training point locations and RSS values
  size_t trainlocations = 0;
  while (std::getline(train_in, line)) {
    std::vector<SolverWorldModel::AttrUpdate> train_attrs;
    std::vector<string> parts = split(line);
    std::vector<u16string> attr_names{u"location.xoffset", u"location.yoffset", u"location.zoffset"};
    if ((parts.size() - num_dimensions) % 2 != 0) {
      std::cerr<<"Training point does not match the given number of dimensions (line is "+line+")\n";
      return 0;
    }
    //Increment the number of fingerprint locations
    ++trainlocations;
    string train_num = std::to_string(trainlocations);
    u16string train_name = u"evaluation_region.training_point." + u16string(train_num.begin(), train_num.end());

    //Get the x, y, and possibly z locations
    for (size_t i = 0; i < num_dimensions; ++i) {
      SolverWorldModel::AttrUpdate offset{attr_names[i], 0, train_name, vector<uint8_t>()};
      pushBackVal(std::stod(parts[i]), offset.data);
      train_attrs.push_back(offset);
    }

    //Now get receiver names and RSS values
    std::vector<u16string> rx_names;
    std::vector<double> rss_vals;
    //Each receiver name will be based on its ID. (convert from char8 to char16)
    for (size_t i = num_dimensions; i < parts.size(); i+=2) {
      u16string rx_name = u"evaluation_region.landmark." + u16string(parts[i].begin(), parts[i].end());
      rx_names.push_back(rx_name);
      double rss = stod(parts[i+1]);
      rss_vals.push_back(rss);
    }
    //Push the rxid vector into the attribute update
    SolverWorldModel::AttrUpdate trainsensor{u"receivers.vector<sized string>", 0, train_name, vector<uint8_t>()};
    pushBackVal<uint32_t>(rx_names.size(), trainsensor.data);
    for (u16string& rxer : rx_names) {
      pushBackSizedUTF16(trainsensor.data, rxer);
    }
    train_attrs.push_back(trainsensor);

    //Push the rss vector into the attribute update
    SolverWorldModel::AttrUpdate trainvals{u"fingerprint.vector<RSS>", 0, train_name, vector<uint8_t>()};
    pushBackVal<uint32_t>(rss_vals.size(), trainvals.data);
    for (double val : rss_vals) {
      pushBackVal(val, trainvals.data);
    }
    train_attrs.push_back(trainvals);

    //Send this training point's attributes to the world model
    swm.sendData(train_attrs, false);
  }
  train_in.close();

  //TODO Send in the test data one point at a time so that localization will occur
  //TODO Check results of the localization program against the ground truths

  return 0;
}


