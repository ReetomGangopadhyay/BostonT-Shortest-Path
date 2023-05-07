# BostonT-Shortest-Path
# Final Project DS210:

## By: Reetom Gangopadhyay


### Project Description:
The project is written using skills we learned in DS210 to find the shortest path for a Boston Tram (T) ride from one station to another using Dijkstra's algorithm. Every station is defined in the code and is represented as a graph and shows the connections between the stations. It then prompts the user to input the starting and ending stations and computes the shortest path using Dijkstra's algorithm. The output will show a distance in meters and every station you will go through to achieve the shortest possible path when going from one station to another. The distances were gathered using the “Measure distance” feature on google maps where I placed the first marker on the first stop and followed the path of the line to get a distance between the two stops. The data is contained in the main file.

#### How to Run:
To run the project, follow the steps below:
1. Clone the project from the repository.
2. Run the command "cargo run" to execute the code.
3. Enter the starting and ending stations for the Boston T trip you want to go on when prompted by the program. The input should match the exact punctuation of the punctuation of the stops on this map at https://cdn.mbta.com/sites/default/files/2022-12/2022-12-12-subway-map-v37f.pdf 

### Output:
After entering the starting and ending stations, the program computes the shortest path and prints it to the console. The output displays the path taken from the starting station to the ending station, along with the distance in meters. If the entered stations are not valid or there is no path between them, the system will exit the program and tell the user that the station is not valid, or that there is no path between the two stops.
 
#### Sample Program Output:

Enter the start stop: Alewife
Enter the end stop: Wonderland

The shortest distance between Alewife and Wonderland has distance in meters and path from start to finish (18937, ["Alewife", "Davis", "Porter", "Harvard", "Central", "Kendall/MIT", "Charles/MGH", "Park St", "Gov't Center", "State", "Aquarium", "Maverick", "Airport", "Wood Island", "Orient Heights", "Suffolk Downs", "Beachmont", "Revere Beach", "Wonderland"])

During the implementation of this project, I discovered a few interesting things. First, I noticed that the T network can be complicated to work with and that there are several paths you can follow to get to another location. I believe that this is for redundancy so that if one line shuts down there are other paths to get to where you need to go in the middle of the city. There are also several edge cases where there are “Rapid Transit transfer stations” that have multiple lines running through them. This made it challenging to implement the algorithm efficiently while also handling different edge cases.

I then implemented a well-known and widely used algorithm, Dijkstra's algorithm, to find the shortest path between any two stations. 

To implement it you must first start at the initial position and checking all of the adjacent vertices, while keeping track of all of the distances from any of the initial vertices to each of the adjacent vertices. Following the discovery of the first vertex, it will then move to the next level of vertices and check for the smallest possible distance and will continue with this method until reaching the final destination. This algorithm almost always guarantees the shortest possible path for a graph.

I also learned about how to handle user input that may not be satisfactory. This is a skill which I know well through python, but have never implemented in Rust. So I had to figure out a solution to this problem as efficiently as I could. Finally, I found it interesting to see the output of the algorithm and how it navigates through the network of stations to find the shortest path. It was also satisfying to see that the algorithm was able to handle different cases and produce accurate results.
