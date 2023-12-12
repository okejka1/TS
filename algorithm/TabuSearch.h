

#ifndef TS_TABUSEARCH_H
#define TS_TABUSEARCH_H


#include "../structures/Graph.h"
#include "../structures/TabuList.h"
#include <iostream>
#include <vector>

using namespace std;
class TabuSearch {
public:
    TabuList tabuList;
    Graph g;
    int *currentSolution;
    int *bestSolution;
    int *neighborSolution;
    int neighborSolutionCost;

    int currentSolutionCost;
    int bestSolutionCost;
    int iterationsSinceChange;
    int numberOfCities;
    std::vector<std::pair<int*, std::pair<int, int>>> neighborSolutions; // vector that stores Neigbour solutions with pair of swapped cities








    // methods
    TabuSearch(Graph &graph);
    void generateGreedySolution();
    void printSolution(int * path);
    int calculateSolutionCost(int *path);
    void generateNeighborSolutions(int numSolutions);
    void generateRandomSolution();

    std::pair<int*, std::pair<int, int>> selectBestAdmissibleSolution();
    void TSSolver(int time);


};


#endif //TS_TABUSEARCH_H
