package Q2;

import java.io.*;
import java.util.*;

public class main {
    // flags to select which algorithm
    final static boolean ASTAR = true;
    final static boolean GREEDY = false;
    final static boolean DIAGONALS_ALLOWED = false;

    
    final static String fileName = "pathfinding_a.txt"; //filename

    public static void main(String[] args) {
        // get contents from file
        ArrayList<ArrayList<Node>> grid = new ArrayList<>(); // used to store the initial grid
        Node startNode = null;
        Node goalNode = null;
        // loop through each character and add it to the grid
        try (BufferedReader br = new BufferedReader(new FileReader(fileName))) {
            int yCoordinate = 0; //keeps track of which yPos coordinate we're on
            for (String line; (line = br.readLine()) != null; ) { // loop through each line
                if (!line.equals("")) {
                    ArrayList<Node> row = new ArrayList<>();
                    for (int xCoordinate = 0; xCoordinate < line.length(); xCoordinate++) {// loop through each character in the line
                        char c = line.charAt(xCoordinate);
                        Node tempNode = new Node(c, xCoordinate, yCoordinate);
                        if (c == 'S') {
                            startNode = tempNode;
                        } else if (c == 'G') {
                            goalNode = tempNode;
                        }
                        row.add(tempNode);
                    }
                    grid.add(row);
                    yCoordinate++;
                } else {
                    // TODO when there's more than one grid...
                    System.out.println("Another grid...");
                    break;
                }
            }
        } catch (Exception e) {
            System.out.println("Could not find " + fileName);
            e.printStackTrace();
            System.exit(1);
        }

        // submitted gridList
        System.out.println();
        System.out.println("initial grid:");
        printGrid(grid);

        ArrayList<ArrayList<Node>> result; // used to store the resulting grid
        // A* Algorithm
        result = pathFinding(grid, startNode, goalNode, ASTAR, DIAGONALS_ALLOWED);
        System.out.println();
        System.out.println("A* Algorithm");
        printGrid(result);

        // Greedy Algorithm
        result = pathFinding(grid, startNode, goalNode, GREEDY, DIAGONALS_ALLOWED);
        System.out.println("Greedy Algorithm");
        printGrid(result);


    }

    private static ArrayList<ArrayList<Node>> pathFinding(ArrayList<ArrayList<Node>> grid, Node startNode, Node goalNode, boolean algorithm, boolean diagonals) {
        // deep copy grid
        ArrayList<ArrayList<Node>> newGrid = new ArrayList<>();
        for (ArrayList<Node> yPos : grid) {
            ArrayList<Node> row = new ArrayList<>();
            for (Node xPos : yPos) {
                row.add(new Node(xPos));
            }
            newGrid.add(row);
        }

        // deep copy start and goal nodes
        Node newStartNode = new Node(startNode);
        Node newGoalNode = new Node(goalNode);

        // the set of nodes to be evaluated (heap)
        Comparator<Node> comparator = Comparator.comparingInt(Node::getF);
        PriorityQueue<Node> open = new PriorityQueue<>(newGrid.size(), comparator);

        // set of nodes already evaluated
        ArrayList<Node> closed = new ArrayList<>();

        // set g and h values, and add the start node to the OPEN set
        newStartNode.setG(0);
        newStartNode.setH(calcDistance(newStartNode, newGoalNode, diagonals));
        open.add(newStartNode);

        while (open.size() > 0) {
            Node current = open.poll(); // current = lowest f cost in open set, and remove current from the set
            if (current.equals(newGoalNode)) {
                // path has been found
                // reconstruct path
                Node tempNode = current.getParent();
                while (!tempNode.equals(newStartNode)) {
                    tempNode.setType('P');
                    tempNode = tempNode.getParent();
                }
                break;
            }
            closed.add(current); // add current to closed set

            // loops through neighbouring nodes and checks if that node is better or not
            ArrayList<Node> neighbourList = new ArrayList<>();
            neighbourList.add(newGrid.get(current.getCoordinate().yPos).get(current.getCoordinate().xPos - 1)); //left
            neighbourList.add(newGrid.get(current.getCoordinate().yPos - 1).get(current.getCoordinate().xPos)); // top
            neighbourList.add(newGrid.get(current.getCoordinate().yPos).get(current.getCoordinate().xPos + 1)); // right
            neighbourList.add(newGrid.get(current.getCoordinate().yPos + 1).get(current.getCoordinate().xPos)); // bot
            if (diagonals) {
                // only add these if we're allowed to move diagonally
                neighbourList.add(newGrid.get(current.getCoordinate().yPos - 1).get(current.getCoordinate().xPos - 1)); //topleft
                neighbourList.add(newGrid.get(current.getCoordinate().yPos - 1).get(current.getCoordinate().xPos + 1)); // top right
                neighbourList.add(newGrid.get(current.getCoordinate().yPos + 1).get(current.getCoordinate().xPos - 1)); // bottom left
                neighbourList.add(newGrid.get(current.getCoordinate().yPos + 1).get(current.getCoordinate().xPos + 1)); // bottom right
            }
            for (Node neighbour : neighbourList) {
                // don't scan node that can't be traversed, or have already been scanned
                if (neighbour.getType() == 'X' || closed.contains(neighbour)) {
                    continue; // go to next neighbour
                }

                if (algorithm) {
                    // A* Algorithm
                    // neighbour is better, or neighbour is not in open, update neighbour
                    int tempNeighbourGCost = current.getG() + calcDistance(current, neighbour, diagonals);
                    if (tempNeighbourGCost < neighbour.getG() || !open.contains(neighbour)) {
                        neighbour.setParent(current);
                        neighbour.setG(tempNeighbourGCost);
                        neighbour.setH(calcDistance(neighbour, newGoalNode, diagonals));
                    }
                    // add neighbour to open if not already in there
                    if (!open.contains(neighbour)) {
                        open.add(neighbour);
                    }
                } else {
                    // Greedy Algorithm
                    // neighbour is better, or neighbour is not in open, update neighbour
                    int tempHCost = calcDistance(neighbour, newGoalNode, diagonals);
                    if (tempHCost < neighbour.getH() || !open.contains(neighbour)) {
                        neighbour.setParent(current);
                        neighbour.setH(tempHCost);
                    }
                    // add neighbour to open if not already in there
                    if (!open.contains(neighbour)) {
                        open.add(neighbour);
                    }
                }
            }

        }

        return newGrid;
    }

    /**
     * Calculates the shortest path from node1 to node2
     *
     * @param n1        node1
     * @param n2        node2
     * @param diagonals flag to determine if you can move diagonally
     * @return distance
     */
    private static int calcDistance(Node n1, Node n2, boolean diagonals) {
        if (diagonals) {
            int distY = Math.abs(n1.getCoordinate().yPos - n2.getCoordinate().yPos);
            int distX = Math.abs(n1.getCoordinate().xPos - n2.getCoordinate().xPos);

            if (distX > distY) {
                return 14 * distY + 10 * (distX - distY);
            } else {
                return 14 * distX + 10 * (distY - distX);
            }
        } else {
            return Math.abs(n1.getCoordinate().yPos - n2.getCoordinate().yPos) + Math.abs(n1.getCoordinate().xPos - n2.getCoordinate().xPos);
        }

    }

    private static void printGridList(ArrayList<ArrayList<ArrayList<Node>>> gridList) {
        for (ArrayList<ArrayList<Node>> grid : gridList) {
            printGrid(grid);
            System.out.println();
        }
    }

    private static void printGrid(ArrayList<ArrayList<Node>> grid) {
        for (ArrayList<Node> yPos : grid) {
            for (Node xPos : yPos) {
                System.out.print(xPos.getType());
            }
            System.out.println();
        }
    }

}