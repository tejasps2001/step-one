package movement;

import core.Coord;
import java.util.*;
import java.io.*;
import java.awt.Color;

import gui.DTNSimGUI;

// Node for RR Tree
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

    // Inner class to hold planning results before committing
    class PlannedSegment {
        Path path;
        Node nodeToCommit;
        boolean isFinalPath = false;
    }

    // GDRRT Parameters
    private double delta;// Tree extension distance
    private double deltaInit = 20.0; // Initial extension value
    private double deltaMin = 5.0; // Minimum extension value for convergence
    private double distGlobMin = 5.0; // Threshold for fine-tuning convergence
    private double rNear = 25.0; // Radius for finding neighbor nodes
    private final double k = 30.0; // Increase in step-size to escape obstacle
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
    
    private static List<java.awt.geom.Line2D.Double> obstacleLines = null;
    private static List<Path> obstaclePaths = null;
    private static List<Color> pathColors = null;
    private static String cachedObstacleFilePath = null;
    private double droneBuffer = 0.5; // Radius of the drone's boundary

    public GDRRTPlanner(String obstacleFilePath) {
        this.obstacleFilePath = obstacleFilePath;
    }

    // Constructor to see the boundary of obstacles
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

    public PlannedSegment planNextSegment() {
        if (!isInitialized) return null;
        
        Random rand = new Random();

        int maxNodesLimit = 1000; // Allow massive expansion if trapped
        int maxIterations = maxNodesLimit * 10; // Prevent infinite loops if completely trapped
        int iterations = 0;

        int nodesInBatch = 0;
        Node finalGoalNode = null;

        // Expansion Phase
        while (nodesInBatch < maxNodesLimit && iterations < maxIterations) {
            iterations++;
            // Smart sampling based on posTemp and range d
            Coord xRand = smartSample(posTemp, delta, rand);
            // Find node nearest to the sample in the tree
            Node xNearest = findNearest(tree, xRand);
            // Create new node via Steer function
            Coord newCoord = steer(xNearest.getLocation(), xRand, delta);

            // Collision check
            if (isCollisionFree(xNearest.position, newCoord)) {
                if (escapeCounter > 2) delta = deltaInit;

                double distNew = newCoord.distance(goal);
                // Update closest node to goal
                if (distNew < distMin || escapeCounter < 3) {
                    posTemp = newCoord;
                    distMin = distNew;
                }
                // Reduce step size if very close to target
                if (distMin < distGlobMin) delta = deltaMin;

                Node xNew = new Node(newCoord, xNearest.getCost() + xNearest.getLocation().distance(newCoord), xNearest);

                // Rewiring the tree and parent selection
                List<Node> xNearNodes = findNear(tree, xNew.getLocation(), rNear);
                chooseParent(xNew, xNearNodes);
                tree.add(xNew);
                nodesInBatch++;
                escaping = false;

                if (xNew.parent != null) xNew.parent.children.add(xNew);
                rewire(tree, xNew, xNearNodes);

                // Check if goal is reached
                if (xNew.getLocation().distance(goal) < delta) {
                    finalGoalNode = new Node(goal, xNew.getCost() + xNew.getLocation().distance(goal), xNew);
                    break;
                }    
                
                // If enough sampling is done, check if we broke out of the local minimum
                if (nodesInBatch >= 25 && nodesInBatch % 25 == 0) {
                    Node currentBest = findNearest(tree, goal);
                    if (currentBest != root && currentBest.getLocation().distance(goal) < root.getLocation().distance(goal) - 1.0) {
                        break;
                    }
                }
            } else {
                // Obstacle avoidance
                delta += k;
                escaping = true;
                escapeCounter = 0;
            }
        }

        PlannedSegment plannedSegment = new PlannedSegment();

        if (finalGoalNode != null) {
            plannedSegment.path = constructPathObject(finalGoalNode);
            plannedSegment.nodeToCommit = finalGoalNode;
            plannedSegment.isFinalPath = true;
            return plannedSegment;
        }

        // Find the best path segment
        Node bestNode = findNearest(tree, goal);
        if (bestNode == root) {
            if (tree.size() > 1) {
                // Local minimum: escape by picking the furthest explored node from the root
                bestNode = tree.stream()
                        .max(Comparator.comparingDouble(n -> n.getLocation().distance(root.getLocation())))
                        .orElse(tree.get(tree.size() - 1));
            } else {
                return null;
            }
        }

        List<Node> pathSegmentNodes = constructPathList(bestNode);
        if (pathSegmentNodes.size() > 1) {
            Node committedNode = pathSegmentNodes.get(1);
            
            Path segment = new Path(0.5);
            segment.addWaypoint(root.getLocation());
            segment.addWaypoint(committedNode.getLocation());

            plannedSegment.path = segment;
            plannedSegment.nodeToCommit = committedNode;
            return plannedSegment;
        }

        return null;
    }

    public void commit(PlannedSegment segment) {
        if (segment == null || segment.nodeToCommit == null) return;

        if (segment.isFinalPath) {
            // Goal reached, reset for next time
            isInitialized = false;
            if (gui != null) {
                gui.showPath(segment.path, Color.GREEN);
            }
            return;
        }

        Node committedNode = segment.nodeToCommit;

        // Prune Phase
        root = committedNode;
        root.parent = null;
        Path z = new Path();
        z.addWaypoint(committedNode.position);
        tree = getSubTree(root, z);
        if (gui != null) {
            // gui.showPath(z, Color.PINK);
            gui.showPath(segment.path, Color.GREEN);
            drawObstacles();
        }

        // Update heuristics
        Node nearest = findNearest(tree, goal);
        posTemp = nearest.getLocation();
        distMin = posTemp.distance(goal);
    }

    private void drawObstacles() {
        if (gui != null && obstaclePaths != null && pathColors != null) {
            for (int i = 0; i < obstaclePaths.size(); i++) {
                gui.showPath(obstaclePaths.get(i), pathColors.get(i));
            }
        }
    }

    private Coord smartSample(Coord posTemp, double d, Random rand) {
        // 10% of the time, or if wildly escaping, sample a wide area to clear large obstacles
        // This 10% number allows the algorithm to escap a local minima faster.
        if (rand.nextDouble() < 0.1 || delta > 50.0) {
            double rx = posTemp.getX() + (rand.nextDouble() - 0.5) * 2000.0;
            double ry = posTemp.getY() + (rand.nextDouble() - 0.5) * 2000.0;
            return new Coord(rx, ry);
        }
        // Equation 2: x_rand = (pos_temp - d/2) + rand * d
        double rx = (posTemp.getX() - d / 2) + rand.nextDouble() * d;
        double ry = (posTemp.getY() - d / 2) + rand.nextDouble() * d;
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
    
    private void loadObstacles() {
        if (obstacleLines != null && this.obstacleFilePath != null && this.obstacleFilePath.equals(cachedObstacleFilePath)) {
            return; // Already loaded and cached globally
        }
        obstacleLines = new ArrayList<>();
        obstaclePaths = new ArrayList<>();
        pathColors = new ArrayList<>();
        cachedObstacleFilePath = this.obstacleFilePath;
        if (this.obstacleFilePath == null) return;
        
        String[] files = this.obstacleFilePath.split(",");
        for (String file : files) {
            String filePath = file.trim();
            if (filePath.isEmpty()) continue;
            
            Color pathColor = Color.BLACK;
            if (filePath.contains("facilities.wkt") || 
                filePath.contains("building") || 
                filePath.contains("facility") ||
                filePath.contains("sl_") ||
                filePath.contains("buildings.wkt") || 
                filePath.contains("u_trap")) {
                // Obstacles in one color
                pathColor = new Color(255, 0, 0, 100); // Specific color with alpha to trigger polygon filling
            }
            
            try (BufferedReader reader = new BufferedReader(new FileReader(filePath))) {
                String lineStr;
                // Matches coordinates in WKT formatting, e.g., "100.0 200.0"
                java.util.regex.Pattern p = java.util.regex.Pattern.compile("(-?[0-9.]+)\\s+(-?[0-9.]+)");
                while ((lineStr = reader.readLine()) != null) {
                    java.util.regex.Matcher m = p.matcher(lineStr);
                    List<Coord> points = new ArrayList<>();
                    while (m.find()) {
                        double lon = Double.parseDouble(m.group(1));
                        double lat = Double.parseDouble(m.group(2));
                        // Translate GPS coordinates to meters ONLY if they are near the Hyderabad area.
                        // Otherwise, assume they are already local Cartesian coordinates.
                        if (lon > 77.0 && lon < 80.0 && lat > 16.0 && lat < 19.0) {
                            points.add(new Coord((lon - 78.300) * 100000.0, (17.480 - lat) * 100000.0));
                        } else {
                            points.add(new Coord(lon, lat));
                        }
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
                    
                    obstaclePaths.add(q);
                    pathColors.add(pathColor);
                    
                    if (gui != null) {
                        gui.showPath(q, pathColor);
                    }

                    // Add visual thickness specifically for the campus boundary map
                    if (filePath.contains("uoh_map.wkt")) {
                        int[] offsets = {-10, -5, 5, 10};
                        for (int offset : offsets) {
                            Path pX = new Path();
                            Path pY = new Path();
                            for (Coord pt : points) {
                                pX.addWaypoint(new Coord(pt.getX() + offset, pt.getY()));
                                pY.addWaypoint(new Coord(pt.getX(), pt.getY() + offset));
                            }
                            obstaclePaths.add(pX);
                            pathColors.add(pathColor);
                            obstaclePaths.add(pY);
                            pathColors.add(pathColor);

                            if (gui != null) {
                                gui.showPath(pX, pathColor);
                                gui.showPath(pY, pathColor);
                            }
                        }
                    }
                }
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }

    private double getSegmentToSegmentDistance(java.awt.geom.Line2D.Double l1, java.awt.geom.Line2D.Double l2) {
        if (l1.intersectsLine(l2)) return 0.0;
        
        double d1 = l1.ptSegDist(l2.getP1());
        double d2 = l1.ptSegDist(l2.getP2());
        double d3 = l2.ptSegDist(l1.getP1());
        double d4 = l2.ptSegDist(l1.getP2());
        
        return Math.min(Math.min(d1, d2), Math.min(d3, d4));
    }

    private boolean isCollisionFree(Coord nearest, Coord newNode) {
        if (obstacleLines == null) {
            loadObstacles();
        }
        
        java.awt.geom.Line2D.Double pathSegment = new java.awt.geom.Line2D.Double(
                nearest.getX(), nearest.getY(), newNode.getX(), newNode.getY());
                
        for (java.awt.geom.Line2D.Double obsLine : obstacleLines) {
            if (getSegmentToSegmentDistance(pathSegment, obsLine) <= droneBuffer) {
                return false; // Collision detected
            }
        }
        return true; // No collision
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
