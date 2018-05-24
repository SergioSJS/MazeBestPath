#Maze Best Path
This project have several algorithms of search best path in graphs. See maps directory to take models of maze,  '@' represent the wall and '.' represent free path. To test just pass the parameters via argument like example below.

##algorithm Type
astar = A*
ucs = Uniform Cost Search
bg = Best First Search
ids = Iterative Deepining Search
dls = Depth Limited Search
dfs = Deep First Search
bfs = Breadth First Search

##Heuristic
1 = Manhattan
2 = Octile
3 = Euclidean

###Output
Arg1 = algorithm
Arg2 = map file
Arg3 and Arg4 = Start integer coordenate
Arg5 and Arg6 = Goal integer coordenate
Arg7 = helristic 

```
python GenericSearch.py astar maps/map1.map 247 245 191 97 2
```