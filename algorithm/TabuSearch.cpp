#include <random>
#include "TabuSearch.h"
#include "../utils/Timer.h"
#include <vector>

TabuSearch::TabuSearch(Graph &graph) : tabuList(graph.vertices*10000), g(graph.vertices) {
    g = graph;
    currentSolution = new int[g.vertices];
    neighborSolution = new int[g.vertices];
    bestSolution = new int[g.vertices];
    currentSolutionCost = 0;
    neighborSolutionCost = INT_MAX;
    bestSolutionCost = INT_MAX;
    iterationsSinceChange = 0;
    numberOfCities = g.vertices;


}

void TabuSearch::generateGreedySolution() {
    bool *visited = new bool[numberOfCities];
    for(int i = 0; i < numberOfCities; i++){
        visited[i] = false;
    }
    int currentCity = 0;
    currentSolution[0] = currentCity;
    visited[currentCity] = true;

    for(int i = 1; i < numberOfCities; i++) {
        int nearestCity = -1;
        int minDistance = INT_MAX;

        for(int j = 0; j < numberOfCities; j++) {
            if(!visited[j] && g.edges[currentCity][j] < minDistance){
                nearestCity = j;
                minDistance = g.edges[currentCity][j];
            }
        }
        currentSolution[i] = nearestCity;
        visited[nearestCity] = true;
        currentSolutionCost += minDistance;
        currentCity = nearestCity;
    }
    currentSolutionCost += g.edges[currentCity][currentSolution[0]];

    std::copy(currentSolution,currentSolution + numberOfCities,bestSolution);
    bestSolutionCost = currentSolutionCost;
    printSolution(currentSolution);
    std::cout << "\nbest solution cost: " << bestSolutionCost;
    delete[] visited;
}

void TabuSearch::printSolution(int *solutionPath) {
    for(int city = 0; city < numberOfCities; city++){
        std::cout<< solutionPath[city] << "-> ";
    }
    std::cout << solutionPath[0];


}

int TabuSearch::calculateSolutionCost(int *path) {
   int cost = 0;
   for(int i = 0; i < numberOfCities - 1; i++) {
       cost += g.edges[path[i]][path[i+1]];
   }
   cost += g.edges[path[numberOfCities-1]][path[0]];

   return cost;
}




void TabuSearch::TSSolver(int maxIterations) {
    int currentIteration = 0;
    generateGreedySolution();
    Timer timer;
    timer.start();

    while (currentIteration < maxIterations) {

            generateNeighborSolutions(numberOfCities);


        int previousCost = currentSolutionCost;
        // Choose the best admissible solution and get the move information
        std::pair<int*, std::pair<int, int>> selectedNeighbor = selectBestAdmissibleSolution();
        std::copy(selectedNeighbor.first, selectedNeighbor.first + numberOfCities, currentSolution);
        int city1 = selectedNeighbor.second.first;
        int city2 = selectedNeighbor.second.second;


        currentSolutionCost = calculateSolutionCost(currentSolution);


        // Check if the neighbor solution is better than the current best solution
        if (currentSolutionCost < bestSolutionCost) {
            // Update the global best solution
            std::copy(currentSolution, currentSolution + numberOfCities, bestSolution);
            bestSolutionCost = currentSolutionCost;
            cout << "New best solution:";
            printSolution(bestSolution);
            cout << "New best solution cost = " << bestSolutionCost << "\n";
            // Reset the iterations since change counter
            iterationsSinceChange = 0;
        } else {
            // Increment the iterations since change counter
            iterationsSinceChange++;
        }

        // Push the chosen move to the Tabu List
        tabuList.push(city1, city2);


        currentIteration++;

    }

    // Output the best solution and its cost
    std::cout << "\nBest solution: ";
    printSolution(bestSolution);
    std::cout << "\nBest solution cost: " << bestSolutionCost << std::endl;

    // Output the total execution time
    timer.stop();

}



void TabuSearch::generateNeighborSolutions(int numSolutions) {
    // Clear any previous neighbor solutions
    neighborSolutions.clear();

    // Create the random number generator outside the loop
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> randomVertex(0, numberOfCities - 1);

    for (int k = 0; k < numSolutions; ++k) {
        // Randomly choose two vertices
        int index1 = randomVertex(gen);
        int index2 = randomVertex(gen);

        while (index1 == index2) {
            // Ensure index2 is different from index1
            index2 = randomVertex(gen);
        }

        // Create a copy of the current solution
        std::copy(currentSolution, currentSolution + numberOfCities, neighborSolution);

        // Swap cities at the selected indices in the copy
        std::swap(neighborSolution[index1], neighborSolution[index2]);

        // Add the neighbor solution to the set
        std::pair<int, int> moveCities = std::make_pair(index1, index2);
        neighborSolutions.push_back(std::make_pair(neighborSolution, moveCities));
    }
}




std::pair<int*, std::pair<int, int>> TabuSearch::selectBestAdmissibleSolution() {
    // Initialize variables to track the best admissible solution
    int bestLocalCost = std::numeric_limits<int>::max();
    int* bestLocalSolution = nullptr;
    std::pair<int, int> bestMoveCities;

    // Iterate over the neighbor solutions
    for (const auto& neighbor : neighborSolutions) {
        int* neighborLocalSolution = neighbor.first;
        std::pair<int, int> moveCities = neighbor.second;

        // Calculate the cost of the neighbor solution
        int neighborLocalSolutionCost = calculateSolutionCost(neighborLocalSolution);

        // Check if the move is in the Tabu List
        int city1 = moveCities.first;
        int city2 = moveCities.second;
        bool inTabu = tabuList.findMove(city1, city2);

        // Check aspiration criteria or if it's better than the current best solution
        if (neighborLocalSolutionCost < bestLocalCost) {
            // Update the best admissible solution and its cost
            bestMoveCities = moveCities;
            bestLocalCost = neighborLocalSolutionCost;
            bestLocalSolution = neighborLocalSolution;
        } else {
            // Clean up dynamically allocated memory for non-selected neighbor solutions
            delete[] neighborLocalSolution;
        }
    }

    // Update the Tabu List with the chosen move
    tabuList.push(bestMoveCities.first, bestMoveCities.second);

    // Return the pair of the best admissible solution and the move
    printSolution(bestLocalSolution);
    cout << "\nCost: " << calculateSolutionCost(bestLocalSolution) << "\n";
    return std::make_pair(bestLocalSolution, bestMoveCities);
}


void TabuSearch::generateRandomSolution() {
    bool *visited = new bool[numberOfCities];
    for (int i = 0; i < numberOfCities; i++) {
        visited[i] = false;
    }

    int currentCity = 0;
    currentSolution[0] = currentCity;
    visited[currentCity] = true;


    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> randomVertex(0, numberOfCities - 1);

    for (int i = 1; i < numberOfCities; i++) {
        int nextCity = randomVertex(gen);
        while (visited[nextCity]) {
            // Ensure we haven't visited this city before
            nextCity = randomVertex(gen);
        }

        currentSolution[i] = nextCity;
        visited[nextCity] = true;
    }

    currentSolutionCost = calculateSolutionCost(currentSolution);

    // Clean up memory
    delete[] visited;
}





