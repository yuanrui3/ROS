#include <algorithm>
#include <iostream>
#include <iomanip>
#include <iterator>
#include <vector>
#include <boost/random.hpp>
#include <boost/random/discrete_distribution.hpp>
#include <boost/random/mersenne_twister.hpp>

// Define base_generator as a Mersenne Twister. This is needed only to make the
// code a bit less verbose.
typedef boost::mt19937 base_generator;


// pull_lever has a chance of 1/weight of returning 1.
unsigned int pull_lever(base_generator *gen, double weight) {
  double probabilities[] = {1-weight, weight};
  boost::random::discrete_distribution<> dist(probabilities);
  return dist(*gen);
}

// argmax returns the index of maximum element in vector v.
template<class T>
size_t argmax(const std::vector<T>& v){
  return std::distance(v.begin(), std::max_element(v.begin(), v.end()));
}


int main(int argc, char* argv[]) {
  unsigned int runs = 0;
  // Probability of winning for each bandit. Change this variable to experiment
  // with different probabilities of winning.
  std::vector<double> p{0.25, 0.45, 0.55};

  // Number of trials per bandit
  auto trials = std::vector<unsigned int>(p.size());
  // Number of wins per bandif
  auto wins = std::vector<unsigned int>(p.size());
  // Beta distributions of the priors for each bandit
  std::vector<boost::random::beta_distribution<> > prior_dists;
  // Initialize the prior distributions with alpha=1 beta=1
  for (size_t i = 0; i < p.size(); i++) {
    prior_dists.push_back(boost::random::beta_distribution<>(1, 1));
  }
  // gen is a Mersenne Twister random generator. We initialzie it here to keep
  // the binary deterministic.
  base_generator gen;
  for (unsigned int i = 0; i < runs; i++) {
    std::vector<double> priors;
    // Sample a random value from each prior distribution.
    for (auto& dist : prior_dists) {
      priors.push_back(dist(gen));
    }
    // Select the bandit that has the highest sampled value from the prior
    size_t chosen_bandit = argmax(priors);
    trials[chosen_bandit]++;
    // Pull the lever of the chosen bandit
    wins[chosen_bandit] += pull_lever(&gen, p[chosen_bandit]);

    // Update the prior distribution of the chosen bandit
    auto alpha = 1 + wins[chosen_bandit];
    auto beta = 1 + trials[chosen_bandit] - wins[chosen_bandit];
    prior_dists[chosen_bandit] = boost::random::beta_distribution<>(alpha, beta);
  }

  auto sp = std::cout.precision();
  std::cout << std::setprecision(3);
  for (size_t i = 0; i < p.size(); i++) {
    std::cout << "Bandit " << i+1 << ": ";
    double empirical_p = double(wins[i]) / trials[i];
    std::cout << "wins/trials: " << wins[i] << "/" << trials[i] << ". ";
    std::cout << "Estimated p: " << empirical_p << " ";
    std::cout << "Actual p: " << p[i] << std::endl;
  }
  std::cout << std::endl;
  auto expected_optimal_wins = *std::max_element(p.begin(), p.end()) * runs;
  std::cout << std::setprecision(sp);
  std::cout << "Expected number of wins with optimal strategy: " << expected_optimal_wins << std::endl;
  std::cout << "Actual wins: " << std::accumulate(wins.begin(), wins.end(), 0) << std::endl;

  return(0);
}