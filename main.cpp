#include <iostream>
#include <string>
#include "structures/Graph.h"
#include "algorithm/TabuSearch.h"

using namespace std;

void test(){
    Graph graph(1);
    string fileName = "ftv47.atsp";
    graph.readGraphDirected(fileName);
    TabuSearch ts(graph);
    ts.generateGreedySolution();
    ts.generateNeighborSolutions(graph.vertices * 100000);
    ts.selectBestAdmissibleSolution();





};
void run(){
    cout << "\nTABU SEARCH -- ATSP\n";
    int option, optimalCost;

    string fileName;
    Graph graph(1);
    do {
        cout << "1.Load graph from file\n";
        cout << "2.Test tabu search\n";
        cout << "3.Change stop condition\n";
        cout << "4.Chose neighbourhood\n";
        cout << "5.Automatic tester\n";
        cout << "6.Exit\n";
        cout << "Choose an option:";


        cin >> option;
        cin.clear(); //This corrects the stream.
        cin.ignore(); //This skips the left over stream data.


        switch(option){
            case 1:
                do {
                    cout << "Choose a file that you want to conduct test on\n";
                    cout << "1.ftv47.atsp\n";
                    cout << "2.ftv170.atsp\n";
                    cout << "3.rbg403.atsp\n";
                    cin >> option;
                    cin.clear();
                    cin.ignore();

                }while(option < 1 || option > 3);
                if(option == 1) {
                    fileName = "ftv47.atsp";
                    optimalCost = 1776;
                }
                if(option == 2) {
                    fileName = "ftv170.atsp";
                    optimalCost = 2755;
                }
                if(option == 3) {
                    fileName = "rbg403.atsp";
                    optimalCost = 2465;
                }
                graph.readGraphDirected(fileName);
                break;
            case 2:
                break;
            case 3:
                break;
            case 4:
                break;
            case 5:
                break;
            case 6:
                break;
            default:
                cout << "Invalid input\n\n";
                break;

        }

    }while(option != 6);

}
int main() {
    //run();
    test();
}
