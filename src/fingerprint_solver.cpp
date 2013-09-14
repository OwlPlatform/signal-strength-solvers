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
 * @file fingerprint_solver.cpp
 * Offer the standard deviations of the RSS values of transmitters as
 * transient data to the world model.
 * Offer both averaged variances (per transmitter) and link variances for
 * each unique transmitter -> receiver link.
 * Also offer median RSS and average RSS of transmitter->receiver links.
 *
 * @author Bernhard Firner
 ******************************************************************************/

#include <algorithm>
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

#include <owl/solver_aggregator_connection.hpp>
#include <owl/solver_world_connection.hpp>
#include <owl/netbuffer.hpp>
#include <owl/sample_data.hpp>
#include <owl/world_model_protocol.hpp>

using std::deque;
using std::get;
using std::map;
using std::pair;
using std::tuple;
using std::u16string;
using std::vector;

using world_model::grail_time;

///Definitions to describe radio links.
typedef tuple<unsigned char, TransmitterID, ReceiverID> Link;
typedef pair<unsigned char, TransmitterID> UniqueTxer;
typedef pair<Timestamp, float> Signal;

///Convert a link to a ID for the world model
std::u16string linkToUString(Link& link) {
  std::string str = std::to_string(get<0>(link)) + "." +
    std::to_string(get<1>(link).lower) + "." + std::to_string(get<2>(link).lower);
  return u16string(str.begin(), str.end());
}

///Convert a transmitter to an ID of the form <physical layer>.<transmitter id>
std::u16string txerToUString(UniqueTxer& ut) {
  std::string str = std::to_string(ut.first) + "." + std::to_string(ut.second.lower);
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

int main(int ac, char** av) {
  if (ac == 2 and std::string(av[1]) == "-?") {
    std::cout<< "name: Signal Strength Fingerprint Solver\n";
    std::cout<< "arguments: aggregator agg_solver worldmodel wm_solver time_interval window_size\n";
    std::cout<< "description: Provides signal statistics as transient types.\n";
    std::cout<< "provides: link variance\n";
    std::cout<< "provides: average variance\n";
    std::cout<< "provides: link average\n";
    std::cout<< "provides: link median\n";
    return 0;
  }

  if (6 > ac) {
    std::cerr<<"This program needs 6 arguments:\n";
    std::cerr<<"\tclient [<aggregator ip> <aggregator port>]+ <world model ip> <world model port> time_interval window_size\n";
    std::cerr<<"Any number of aggregator ip/port pairs may be provided to connect to multiple servers.\n";
    std::cerr<<"The time interval sets the interval between results in milliseconds.\n";
    std::cerr<<"The window size sets the history (in milliseconds) used in each result.\n";
    return 0;
  }

  //Grab the ip and ports for the servers and aggregator
  std::vector<SolverAggregator::NetTarget> servers;
  for (int s_num = 1; s_num < ac - 4; s_num += 2) {
    std::string server_ip(av[s_num]);
    uint16_t server_port = std::stoi(std::string((av[s_num + 1])));
    servers.push_back(SolverAggregator::NetTarget{server_ip, server_port});
  }
  //World model IP and port
  std::string wm_ip(av[ac-4]);
  int wm_port = std::stoi(std::string((av[ac-3])));
  //Time interval and window size - throws an exception if the user
  //input cannot be interpreted as floats.
  grail_time time_interval = std::stoll(std::string(av[ac-2]));
  grail_time window_size = std::stoll(std::string(av[ac-1]));
  std::cerr<<"Using time interval "<<time_interval<<" and window size "<<window_size<<'\n';


  //Set up the solver world model connection;
  std::string origin = "fingerprint_solver (time interval "+
    std::to_string(time_interval)+", window size "+std::to_string(window_size)+")";
  //Provide variance as a transient type
  //Link variance is between a transmitter and a receiver.
  //Average variance is the average of all link variances for a transmitter
  std::vector<std::pair<u16string, bool>> type_pairs{{u"link variance", true},
                                                     {u"average variance", true},
                                                     {u"link average", true},
                                                     {u"link median", true}};
  SolverWorldModel swm(wm_ip, wm_port, type_pairs, u16string(origin.begin(), origin.end()));
  if (not swm.connected()) {
    std::cerr<<"Could not connect to the world model - aborting.\n";
    return 0;
  }

  //Create a place to store signal values received from the aggregator
  map<Link, deque<Signal>> signals;
  //We will run the aggregator connection in another thread so we
  //need a mutex to control access to the signals map.
  std::mutex signal_mutex;

  //Get all data from all physical layers (0 represents any physical layer)
  aggregator_solver::Rule everything_rule;
  everything_rule.physical_layer = 0;
  //Request data for at most twice the time interval.
  everything_rule.update_interval = time_interval / 2.0;
  auto packet_callback = [&](SampleData& s) { storeLinkRSS(s, signals, std::ref(signal_mutex));};
  SolverAggregator aggregator(servers, packet_callback);
  aggregator.updateRules(aggregator_solver::Subscription{everything_rule});

  //Now process signal values into variance every time_interval milliseconds
  grail_time last_time = world_model::getGRAILTime();
  //Sleep for half a window to allow samples to arrive.
  usleep(window_size * 1000 / 2.0);
  while (1) {
    grail_time cur_time = world_model::getGRAILTime();
    //Sleep if there is time until the next interval
    if (cur_time < last_time + time_interval) {
      grail_time interval = last_time + time_interval - cur_time;
      //GRAIL time is in milliseconds to multiply by 1000
      usleep(interval * 1000);
    }
    //Calculate the variance of all transmitters.
    //The variance that each receiver observes is averaged to obtain the final value.
    //A receiver must see at least 3 samples to be used in the variance calculation.
    last_time = world_model::getGRAILTime();
    grail_time cutoff_time = last_time - window_size;
    //Get the variance for each transmitter/receiver link and store them by transmitter
    map<UniqueTxer, vector<double>> txer_variances;
    //Store link variances while doing this
    vector<SolverWorldModel::AttrUpdate> solns;
    {
      std::unique_lock<std::mutex> lck(signal_mutex);
      for (auto I = signals.begin(); I != signals.end() ; ++I) {
        //Process all of the signals from this link
        UniqueTxer ut{std::get<0>(I->first), std::get<1>(I->first)};
        Link link = I->first;
        deque<Signal>& dq = I->second;
        //Remove data that is too old. This also stops the dequeue from growing too large
        while (not dq.empty() and dq.front().first < cutoff_time) {
          dq.pop_front();
        }
        //Find the signal statistics and store them if there are enough samples.
				//At least 1 sample to find a median and average.
				if (1 <= dq.size()) {
					double avg = std::accumulate(dq.begin(), dq.end(), 0.0,
							[&](double a, Signal b){return a + (b.second/dq.size());});
					//Make a link average solution
					{
						SolverWorldModel::AttrUpdate avg_soln{u"link average", last_time, linkToUString(link), vector<uint8_t>()};
						pushBackVal(avg, avg_soln.data);
						solns.push_back(avg_soln);
					}
					//Make a link median solution
					{
						std::vector<float> rss_vals(dq.size());
						std::transform(dq.begin(), dq.end(), rss_vals.begin(), [&](const Signal& s){return s.second;});
						std::sort(rss_vals.begin(), rss_vals.end());
						double median = rss_vals[rss_vals.size() / 2];
						if (rss_vals.size() % 2 == 0) {
							median = (median + rss_vals[1 + rss_vals.size()/2])/ 2.0;
						}
						SolverWorldModel::AttrUpdate median_soln{u"link median", last_time, linkToUString(link), vector<uint8_t>()};
						pushBackVal(median, median_soln.data);
						solns.push_back(median_soln);
					}
					if (3 <= dq.size()) {
						double ssquares = std::accumulate(dq.begin(), dq.end(), 0.0,
								[&](double a, Signal b){return a + pow(avg - b.second, 2.0);});
						double variance = ssquares / (dq.size() - 1);
						{
							SolverWorldModel::AttrUpdate soln{u"link variance", last_time, linkToUString(link), vector<uint8_t>()};
							pushBackVal(variance, soln.data);
							solns.push_back(soln);
							txer_variances[ut].push_back(variance);
						}
					}
				}
      }
    }
    //Now average the variances for each transmitter and turn the values
    //into solution types.
    for (auto I = txer_variances.begin(); I != txer_variances.end(); ++I) {
      UniqueTxer ut = I->first;
      SolverWorldModel::AttrUpdate soln{u"average variance", last_time, txerToUString(ut), vector<uint8_t>()};
      double avg = getAverage(I->second);
      pushBackVal(avg, soln.data);
      solns.push_back(soln);
    }
    //Do not create URIs for these entries, just send the data
    if (not solns.empty()) {
      std::cerr<<"Updating "<<solns.size()<<" attributes.\n";
      swm.sendData(solns, false);
    }
  }

  return 0;
}

