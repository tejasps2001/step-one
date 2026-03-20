package movement;

import core.Coord;
import java.util.*;
import java.io.*;
import java.awt.Color;

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
    
    private List<Node> tree = new ArrayList<>();
    private Node root;
    private Coord goal;
    private boolean isInitialized = false;
    private Coord posTemp;
    private double distMin;
    private List<java.awt.geom.Line2D.Double> obstacleLines = null;

    // TODO: Below constructor is the long-term solution
    public GDRRTPlanner(String obstacleFilePath) {
        this.obstacleFilePath = obstacleFilePath;
    }

    // Below const only to see the boundary of obstacles
    public GDRRTPlanner(DTNSimGUI gui) {
        GDRRTPlanner.gui = gui;
    }

public boolean isInitialized() {
        return isInitialized;
    }

    public void init(Coord start, Coord goal) {
        this.tree.clear();
        this.root = new Node(start, 0);
        this.tree.add(root);
        this.goal = goal;
        this.posTemp = start;
        this.distMin = start.distance(goal);
        this.delta = deltaInit;
        this.escapeCounter = 3;
        this.isInitialized = true;
    }

    public Path getNextPath() {
        if (!isInitialized) return null;
        
        Random rand = new Random();
        Path z = new Path();

        int batchSize = 100; // Time devoted to planning phase (nodes per commit)


        int nodesInBatch = 0;
        boolean goalReached = false;
        Node finalGoalNode = null;

        // Expansion Phase
        while (nodesInBatch < batchSize) {
            // Line 14: Smart sampling based on posTemp and range d [cite: 361]
            Coord xRand = smartSample(posTemp, delta, rand);
            // Line 5: Find nearest node in tree [cite: 350]
            Node xNearest = findNearest(tree, xRand);
            // Line 6: Create new node via Steer function [cite: 351]
            Coord newCoord = steer(xNearest.getLocation(), xRand, delta);

            // Line 7: Collision check
            if (isCollisionFree(xNearest.position, newCoord)) {
                if (escapeCounter > 2) delta = deltaInit;

                double distNew = newCoord.distance(goal);
                // Lines 9-11: Update closest node to goal
                if (distNew < distMin || escapeCounter < 3) {
                    posTemp = newCoord;
                    distMin = distNew;
                }
                // Lines 12-13: Reduce step size if very close to target
                if (distMin < distGlobMin) delta = deltaMin;

                Node xNew = new Node(newCoord, xNearest.getCost() + xNearest.getLocation().distance(newCoord), xNearest);

                // Lines 15-17: RRT* Rewiring and parent selection
                List<Node> xNearNodes = findNear(tree, xNew.getLocation(), rNear);
                chooseParent(xNew, xNearNodes);
                tree.add(xNew);
                nodesInBatch++;
                escaping = false;
                z.addWaypoint(xNew.position);
                if (gui != null) gui.showPath(z);

                if (xNew.parent != null) xNew.parent.children.add(xNew);
                rewire(tree, xNew, xNearNodes);

                // Check if goal is reached
                if (xNew.getLocation().distance(goal) < delta) {
                    finalGoalNode = new Node(goal, xNew.getCost() + xNew.getLocation().distance(goal), xNew);
                    goalReached = true;
                    break;
                }    
            } else {
                // Line 19-21: Obstacle avoidance [cite: 368, 369, 370]
                delta += 30.0; // k constant
                escaping = true;
                escapeCounter = 0;
            }
        }

        if (goalReached && finalGoalNode != null) {
            Path p = constructPathObject(finalGoalNode);
            isInitialized = false; // Goal reached, reset
            return p;
        }

        // Commit Phase: Pick best path and commit to initial portion
        Node bestNode = findNearest(tree, goal);
        if (bestNode == root) {
            return null;
        }

        List<Node> pathSegment = constructPathList(bestNode);
        if (pathSegment.size() > 1) {
            Node committedNode = pathSegment.get(1); // First step after root
            
            Path segment = new Path(0.5); // assuming constant speed
            segment.addWaypoint(root.getLocation());
            segment.addWaypoint(committedNode.getLocation());

            // Prune Phase
            root = committedNode;
            root.parent = null;
            z.addWaypoint(committedNode.position);
            tree = getSubTree(root, z);
            if (gui != null) gui.showPath(z, Color.BLUE);

            // Update heuristics based on surviving tree
            Node nearest = findNearest(tree, goal);
            posTemp = nearest.getLocation();
            distMin = posTemp.distance(goal);
            return segment;
        }

        return null;
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
    
    private Path constructPathObject(Node endNode) {
        Path p = new Path(0.5);
        List<Node> list = constructPathList(endNode);
        for(Node n : list) {
            p.addWaypoint(n.getLocation());
        }
        return p;
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
    private void loadObstacles() {
        obstacleLines = new ArrayList<>();
        if (this.obstacleFilePath == null) return;
        
        try (BufferedReader reader = new BufferedReader(new FileReader(obstacleFilePath))) {
            String lineStr;
            // Matches coordinates in WKT formatting, e.g., "100.0 200.0"
            java.util.regex.Pattern p = java.util.regex.Pattern.compile("(-?[0-9.]+)\\s+(-?[0-9.]+)");
            while ((lineStr = reader.readLine()) != null) {
                java.util.regex.Matcher m = p.matcher(lineStr);
                List<Coord> points = new ArrayList<>();
                while (m.find()) {
                    points.add(new Coord(Double.parseDouble(m.group(1)), Double.parseDouble(m.group(2))));
                }
                
                if (points.isEmpty()) continue;
                
                Path q = new Path();
                for (int i = 0; i < points.size() - 1; i++) {
                    Coord p1 = points.get(i);
                    Coord p2 = points.get(i + 1);
                    obstacleLines.add(new java.awt.geom.Line2D.Double(p1.getX(), p1.getY(), p2.getX(), p2.getY()));
                    q.addWaypoint(p1);
                }
                q.addWaypoint(points.get(points.size() - 1));
                
                if (gui != null) {
                    gui.showPath(q);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
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
    private boolean isCollisionFree(Coord nearest, Coord newNode) {
        if (obstacleLines == null) {
            loadObstacles();
        }

        Coord[] obsArray = coords.toArray(new Coord[0]);
        Path q = new Path();
        for (Coord c : obsArray) {
            q.addWaypoint(c);
        
        java.awt.geom.Line2D.Double pathSegment = new java.awt.geom.Line2D.Double(
                nearest.getX(), nearest.getY(), newNode.getX(), newNode.getY());
                
        for (java.awt.geom.Line2D.Double obsLine : obstacleLines) {
            if (pathSegment.intersectsLine(obsLine)) {
                return false; // Collision detected
            }
        }

        if (gui != null && obsArray.length > 0) {
            gui.showPath(q);
        }
        return true; // No collision
    }
}

    private List<Node> getSubTree(Node root, Path z) {
        List<Node> subTree = new ArrayList<>();
        Queue<Node> q = new LinkedList<>();
        q.add(root);
        while (!q.isEmpty()) {
            Node n = q.poll();
            subTree.add(n);
            z.addWaypoint(n.position);
            for (Node child : n.children) {
                q.add(child);
            }
        }
        return subTree;
    }

    private List<Node> constructPathList(Node goalNode) {
        List<Node> path = new ArrayList<>();
        Node curr = goalNode;
        while (curr != null) {
            path.add(curr);
            curr = curr.parent;
        }
        Collections.reverse(path);
        return path;
    }
}
