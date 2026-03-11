package movement;

import core.Coord;
import java.util.*;
import java.io.*;

import gui.DTNSimGUI;

class Node {
    Set<Node> children;
    Node parent;
    Coord position;
    double cost;

    Node(Coord c, double cost) {
        this.position = c.clone();
        this.cost = cost;
        this.parent = null;
        this.children = new HashSet<Node>();
    }

    Node(Coord c, double cost, Node parent) {
        this.position = c.clone();
        this.cost = cost;
        this.parent = parent;
        this.children = new HashSet<Node>();
    }

    Coord getLocation() {
        return this.position.clone();
    }

    double getCost() {
        return this.cost;
    }

    public String toString() {
        String str = "" + this.position + "-->";
        for (Node i : children) {
            str += "" + i.position;

        }
        return str;
    }
}

public class GDRRTPlanner {

    // Parameters based on the study settings [cite: 547]
    private double delta;// Tree extension distance (δ)
    private double deltaInit = 20.0; // Initial extension value
    private double deltaMin = 5.0; // Minimum extension value for convergence
    private double d = 25.0; // Sampling range diameter
    private double distGlobMin = 5.0; // Threshold for fine-tuning convergence
    private double rNear = 25.0; // Radius for finding neighbor nodes
    // TODO: Must increase maxNodes since sometimes it isn't sufficent
    private int maxNodes = 3000; // Maximum node limit
    private static DTNSimGUI gui;

    private String obstacleFilePath;
    private boolean escaping = false;
    private int escapeCounter = 3;

    // TODO: Below constructor is the long-term solution
    public GDRRTPlanner(String obstacleFilePath) {
        this.obstacleFilePath = obstacleFilePath;
    }

    // Below const only to see the boundary of obstacles
    public GDRRTPlanner(DTNSimGUI gui) {
        GDRRTPlanner.gui = gui;
    }

    /**
     * Implementation of the GDRRT* algorithm [cite: 337]
     */
    private Node[] planPath(Node xInit, Node xGoal) {
        List<Node> tree = new ArrayList<>();
        tree.add(xInit); // Line 2: Initialize tree with root [cite: 340]
        delta = deltaInit;
        Node xCurrent = xInit;
        Coord posTemp = xInit.getLocation(); // Line 3: Temp position for smart sampling [cite: 341]
        double distMin = xInit.getLocation().distance(xGoal.getLocation());

        Random rand = new Random();
        Path z = new Path();

        for (int i = 0; i < maxNodes; i++) {
            // Line 14: Smart sampling based on posTemp and range d [cite: 361]
            Coord xRand = smartSample(posTemp, delta, rand);

            // Line 5: Find nearest node in tree [cite: 350]
            Node xNearest = findNearest(tree, xRand);

            // Line 6: Create new node via Steer function [cite: 351]
            

            
            Coord newCoord = steer(xNearest.getLocation(), xRand, delta);
            

            // Line 7: Collision check (Assumed true for this implementation) [cite: 352]
            if (isCollisionFree(xNearest.position, newCoord)) {
                if (escapeCounter > 2)
                    delta = deltaInit; // Line 8 [cite: 353]
            
                double distNew = newCoord.distance(xGoal.getLocation());

                // Lines 9-11: Update closest node to goal [cite: 354, 355, 357]
                if (distNew < distMin || escapeCounter < 3) {
                    posTemp = newCoord;
                    distMin = distNew;

                }

                // Lines 12-13: Reduce step size if very close to target [cite: 359, 360]
                if (distMin < distGlobMin) {
                    delta = deltaMin;
                }

                Node xNew = new Node(newCoord, xNearest.getCost() + xNearest.getLocation().distance(newCoord),
                        xNearest);

                // Lines 15-17: RRT* Rewiring and parent selection [cite: 362, 363, 366]
                List<Node> xNearNodes = findNear(tree, xNew.getLocation(), rNear);
                chooseParent(xNew, xNearNodes);
                tree.add(xNew);
                escaping = false;
                z.addWaypoint(xNew.position);
                gui.showPath(z);
                xNearest.children.add(xNew);
                rewire(tree, xNew, xNearNodes);

                // Check if goal is reached
                if (xNew.getLocation().distance(xGoal.getLocation()) < delta) {
                    xGoal.parent = xNew;
                    return constructPath(xGoal);
                }
            } else {
                // Line 19-21: Obstacle avoidance [cite: 368, 369, 370]
                delta += 30.0; // k constant
                escaping = true;
                escapeCounter = 0;
                
            }
        }

        return new Node[0]; // No path found
    }

    public Path generatePath(Coord xInit1, Coord xGoal1) {
        Node xInit = new Node(xInit1, 0);
        Node xGoal = new Node(xGoal1, Integer.MAX_VALUE);
        Path p = new Path(0.5);
        Node[] pathArray = new Node[1000];
        // Backtrack from xnearest to root using parent pointers
        pathArray = planPath(xInit, xGoal);
        for (Node waypoint : pathArray)
            p.addWaypoint(waypoint.position);
        return p;
    }

    private Coord smartSample(Coord posTemp, double d, Random rand) {
        // Equation 2: x_rand = (pos_temp - d/2) + rand * d [cite: 318]
        double rx = (posTemp.getX() - d / 2) + rand.nextDouble() * d;
        double ry = (posTemp.getY() - d / 2) + rand.nextDouble() * d;
        // System.out.println("rx is "+rx + "  & ry is"+ ry);
        return new Coord(rx, ry);
    }

    private Coord steer(Coord from, Coord to, double stepSize) {
        double dist = from.distance(to);
        escapeCounter += 1;
        if (dist < stepSize)
            return to;
        double theta = Math.atan2(to.getY() - from.getY(), to.getX() - from.getX());
        return new Coord(from.getX() + stepSize * Math.cos(theta), from.getY() + stepSize * Math.sin(theta));
    }

    private Node findNearest(List<Node> tree, Coord target) {
        return tree.stream().min(Comparator.comparingDouble(n -> n.getLocation().distance(target))).orElse(tree.get(0));
    }

    private List<Node> findNear(List<Node> tree, Coord target, double radius) {
        List<Node> near = new ArrayList<>();
        for (Node n : tree) {
            if (n.getLocation().distance(target) < radius)
                near.add(n);
        }
        return near;
    }

    private void chooseParent(Node xNew, List<Node> nearNodes) {
        for (Node near : nearNodes) {
            double newCost = near.getCost() + near.getLocation().distance(xNew.getLocation());
            if (newCost < xNew.getCost()) {
                if (isCollisionFree(xNew.position, near.position)) {
                    xNew.parent = near;
                    xNew.cost = newCost;

                }
            }
        }
    }

    private void rewire(List<Node> tree, Node xNew, List<Node> nearNodes) {
        for (Node near : nearNodes) {
            double newCost = xNew.getCost() + xNew.getLocation().distance(near.getLocation());
            if (newCost < near.getCost() && isCollisionFree(xNew.position, near.position)) {

                near.parent.children.remove(near);
                near.parent = xNew;
                near.cost = newCost;
                xNew.children.add(near);
            }
        }
    }
private boolean isCollisionFree(Coord nearest, Coord newNode) {
    String filePath = "./src/main/python/WKT.py";
    int exitCode = 0;
    String nearest_newNode = "LINESTRING (" + nearest.getX() + " " + nearest.getY() +
            ", " + newNode.getX() + " " + newNode.getY() + ")";
    ProcessBuilder pb = new ProcessBuilder("python", filePath,
            this.obstacleFilePath, nearest_newNode);
    pb.redirectErrorStream(true);

    try {
        Process process = pb.start();
        StringBuilder output = new StringBuilder();
        try (BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()))) {
            String line;
            while ((line = reader.readLine()) != null) {
                output.append(line);
            }
        }

        renderObstacleBoundaries(output.toString());

        exitCode = process.waitFor();
    } catch (Exception e) {
        e.printStackTrace();
    }
    return (exitCode != 0);
}

private void renderObstacleBoundaries(String outputStr) {
    // Split output by separator to get each obstacle separately
    String[] obstacleGroups = outputStr.split("---");

    for (String group : obstacleGroups) {
        java.util.regex.Pattern p = java.util.regex.Pattern.compile("\\((-?[0-9.]+),\\s*(-?[0-9.]+)\\)");
        java.util.regex.Matcher m = p.matcher(group);
        List<Coord> coords = new ArrayList<>();

        while (m.find()) {
            coords.add(new Coord(Double.parseDouble(m.group(1)), Double.parseDouble(m.group(2))));
        }

        Coord[] obsArray = coords.toArray(new Coord[0]);
        Path q = new Path();
        for (Coord c : obsArray) {
            q.addWaypoint(c);
        }

        if (gui != null && obsArray.length > 0) {
            gui.showPath(q);
        }
    }
}

    private Node[] constructPath(Node goalNode) {
        List<Node> path = new ArrayList<>();
        Node curr = goalNode;
        while (curr != null) {
            path.add(curr);
            curr = curr.parent;
        }
        Collections.reverse(path);
        return path.toArray(new Node[0]);
    }
}
