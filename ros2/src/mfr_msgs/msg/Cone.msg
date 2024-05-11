uint16      id # node id, directed graph
uint16      followed_by # id of previous cone
uint16      following   # id of next cone
float64[]   search_direction # starting from [x,y] ending at here

float64     x # defines x mean if probabilistic approach 
float64     y 
float64     std # std of the observation distances
uint16      count # trivial

string      color
