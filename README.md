# GhostofBirds

Observes the trade-offs between space and time complexity between iterative deepening Conflict Based Search (CBS) and standard CBS.

## Usages

Implements and tests several combinations of iterative deepening A* search and CBS ranging from independent to multi-agent path finding.

## Running the programs

To run CBS or IDCBS:
1. Change the variable in cbs.py on line 6 to True if you want to run IDCBS, and False if you want to run CBS
2. Run the following command: `python run_experiments.py --instance "instances/test_*" --batch --disjoint --repeat X`, with X being the number of sample to take from each test instance

To generate graphs:
1. Rename the names of the result spreadsheets by prepending the algorithm name: "IDCBS" or "CBS"
2. Run the following command: `python graph.py`

## Contributions
Hengyu Cui

Justin Yiu Ming Mok

Don Van