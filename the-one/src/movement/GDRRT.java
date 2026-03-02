// package movement;

// import core.Coord;
// import java.util.*;

// class Node {
//   Set<Node> children;
//   Node parent;
//   Coord position;
//   double cost;

//   Node(Coord c, double cost) {
//     this.position = c.clone();
//     this.cost = cost;
//     this.parent = null;
//     this.children = new HashSet<Node>();
//   }

//   Node(Coord c, double cost, Node parent) {
//     this.position = c.clone();
//     this.cost = cost;
//     this.parent = parent;
//     this.children = new HashSet<Node>();
//   }

//   Coord getLocation() {
//     return this.position.clone();
//   }

//   double getCost() {
//     return this.cost;
//   }
//   public String toString( ) { 
//         String str = ""+this.position + "-->";
//         for( Node i : children){
//           str+=""+ i.position;
    
//         }
//         return str;
//   }
// }

// public class GDRRT {
//   // tree for storing the current path options
//   private static Node tree;

//   // size of the sampling sphere

//   private static double nearRadius = 5;

//   // bounds for the solution space (hard coded for now, calculate it later)
//   private static Coord topLeft = new Coord(100, 100);
//   private static Coord bottomRight = new Coord(1000, 1000);

//   // total no of iterations algorithm runs far
//   // higher means better path
//   private static int totalIterations = 100;

//   public static Path findPath(Coord startLoc, Coord endLoc) {
//     tree = new Node(startLoc, 0);
//     double minDistanceFromGoal = startLoc.distance(endLoc);
//     double delta_init = 0.1 * minDistanceFromGoal;
//     double d = 2 * delta_init;
//     double delta_min = 0.001 * minDistanceFromGoal;
//     double k = 0.01 * minDistanceFromGoal;
//     double delta = delta_init;
//     double globalMinDistance = 2 * delta_init;
//     Coord posTemp = tree.getLocation();
//     Coord rand = sample(d, posTemp);

//     for (int i = 0; i < totalIterations; i++) {
//       Node nearest = findNearest(tree, rand);
//       // System.out.println("" + nearest.position + rand + delta);
//       Node newNode = steer(nearest.position, rand, delta);

//       // TODO: define obstacles from mapBasedMM
//       // not sure what type obstacles should have or if there should be multiple
//       // arguments
//       // TODO: pass the obstacle file from GDRRTMovement
//       if (collisionFree(nearest.position, newNode.position)) {
//         delta = delta_init;
//         double currentDistance = distance(newNode.position, endLoc);
//         if (currentDistance < minDistanceFromGoal) {
//           posTemp = newNode.position;
//           minDistanceFromGoal = currentDistance;
//         }

//         // what is this globalMinDistance exactly?
//         if (minDistanceFromGoal < globalMinDistance) {
//           delta = delta_min;
//         }

//         rand = sample(d, posTemp);
//         List<Node> nearNodes = near(tree, newNode.position, nearRadius);
//         Node minNode = chooseParent(nearNodes, nearest, newNode.position);
//         insertNode(minNode, newNode);
//         System.out.println("Printing the minNode " + minNode);
//       } else {
//         delta = delta + k;
//         rand = sample();
//       }

//     }
//     return generatePath(tree, new Node(endLoc, Integer.MAX_VALUE));
//   }

//   /*
//    * Generates a random coord bounded by topLeft & bottomRight Coord object
//    */
//   private static Coord sample() {
//     // get bounds from topLeft & bottomRight
//     double leftX = topLeft.getX();
//     double topY = topLeft.getY();
//     double rightX = bottomRight.getX();
//     double bottomY = bottomRight.getY();

//     // generate a random (x, y) coordinate within bounds
//     Random randomInstance = new Random();
//     double x = leftX + (rightX - leftX) * randomInstance.nextDouble();
//     double y = topY + (bottomY - topY) * randomInstance.nextDouble();

//     return new Coord(x, y);
//   }

//   /*
//    * Generates a random coord bounded by topLeft & bottomRight Coord object
//    * The sample space is further limited by the hypersphere around posTemp
//    */
//   private static Coord sample(double d, Coord posTemp) {
//     // get bounds from topLeft, bottomRight & posTemp + d
//     double leftX = Math.max(topLeft.getX(), posTemp.getX() - d);
//     double topY = Math.max(topLeft.getY(), posTemp.getY() - d);
//     double rightX = Math.min(bottomRight.getX(), posTemp.getX() + d);
//     double bottomY = Math.min(bottomRight.getY(), posTemp.getY() + d);

//     // generate a random (x, y) coordinate within bounds
//     Random randomInstance = new Random();
//     double x = leftX + (rightX - leftX) * randomInstance.nextDouble();
//     double y = topY + (bottomY - topY) * randomInstance.nextDouble();

//     return new Coord(x, y);
//   }

//   /*
//    * Returns the rand Coord if it is reachable by covering delta distance
//    * from the nearest node in the RR Tree.
//    * If not, returns the furthest Coord in the direction of rand
//    * that delta distance allows to go to.
//    */
//   private static Node steer(Coord nearest, Coord rand, double delta) {
//     double distance = nearest.distance(rand);
//     if (distance <= delta)
//       return new Node(new Coord(rand.getX(), rand.getY()), Integer.MAX_VALUE);

//     double xDistance = rand.getX() - nearest.getX();
//     double yDistance = rand.getY() - nearest.getY();
//     double parts = delta / distance;

//     double x = nearest.getX() + (parts * xDistance);
//     double y = nearest.getY() + (parts * yDistance);

//     return new Node(new Coord(x, y), Integer.MAX_VALUE);
//   }

//   // TODO: implement this
//   // Traversing the tree in Level order way
//   // and try to check for min distance for each node;
//   private static Node findNearest(Node tree, Coord rand) {
//     Node xNearest = null;
//     double distance = Integer.MAX_VALUE;
//     if (tree == null) {
//       return null;
//     }
//     Queue<Node> q = new LinkedList<>();
//     q.add(tree);
//     while (!q.isEmpty()) {
//       Node temp = q.remove();
//       double temp_distance = distance(rand, temp.position);
//       if (temp_distance < distance) {
//         xNearest = temp;
//         distance = temp_distance;
//       }
//       for (Node i : temp.children) {
//         if (i != null) {
//           q.add(i);
//         }

//       }
//     }
//     return xNearest;
//   }

//   /*
//    * Return the Node from the tree which have least cost
//    * from itself to the newNode and
//    * Doing this for checking there is any cases there is another
//    * Node nearer than nearest
//    */
//   private static Node chooseParent(List<Node> nearNodes, Node nearest, Coord newNode) {
//     Node min = nearest;
//     min.cost = distance(min.position, newNode) + min.cost;
//     for (int i = 0; i < nearNodes.size(); i++) {
//       Node temp = nearNodes.get(i);
//       if (temp.cost + distance(temp.position, newNode) > min.cost) {
//         min = temp;
//         min.cost = temp.cost + distance(temp.position, newNode);
//       }
//     }
//     return min;
//   }

//   // Euclidian distance between two points in the space
//   private static double distance(Coord a, Coord b) {
//     return Math.sqrt(Math.pow(b.getX() - a.getX(), 2) + Math.pow(b.getY() - a.getY(), 2));
//   }

//   /*
//    * Inserting the Node as the child of the parent of
//    */
//   public static void insertNode(Node minNode, Node newNode) {
//     minNode.children.add(newNode);
//   }

//   /*
//    * Finds all the nodes in the hypersphere with the newNode as center
//    * and radius rNear
//    */
//   static List<Node> near(Node tree, Coord newNode, double rNear) {
//     List<Node> list = new ArrayList<>();
//     Queue<Node> q = new LinkedList<>();

//     q.add(tree);
//     while (!q.isEmpty()) {
//       Node temp = q.remove();
//       double temp_distance = newNode.distance(temp.position);
//       if (temp_distance < rNear) {
//         list.add(temp);
//       }

//       for (Node i : temp.children) {
//         if (i != null) {
//           q.add(i);
//         }
//       }
//     }

//     return list;
//   }

//   static boolean collisionFree(Coord nearest, Coord newNode) {
//     String filePath = "C:\\Users\\tejas\\Documents\\Class_14\\Project\\Case_Study\\step-one\\the-one\\src\\util\\WKT.py";
//     int exitCode = 0;
//     String obstacle_file_path = "C:\\Users\\tejas\\Documents\\Class_14\\Project\\Case_Study\\step-one\\step-one-main\\samples\\disaster_scenario\\target_roads_final.wkt";
//     String nearest_newNode = "LINESTRING (" + nearest.getX() + " " + nearest.getY() +
//                               ", " + newNode.getX() + newNode.getY() + ")";
//     ProcessBuilder pb = new ProcessBuilder("python", filePath,
//         obstacle_file_path, nearest_newNode);
//     try {
//       Process process = pb.start();
//       exitCode = process.waitFor();
//     } catch (Exception e) {
//       e.printStackTrace();
//     }
//     return (exitCode != 0);
//   }

//   static Path generatePath(Node tree, Node xtarget) {
//     Path p = new Path();
//     List<Node> pathArray = new ArrayList<Node>();
//     // Backtrack from xnearest to root using parent pointers
//     pathArray = dfs(tree, pathArray, xtarget);
//     for(Node waypoint : pathArray)
//       p.addWaypoint(waypoint.position);
//     return p;
//   }

//   static List<Node> dfs(Node tree, List<Node> pathSoFar, Node xtarget) {
//     if(tree == null) return null;
//     if(tree == xtarget) return pathSoFar;
//     pathSoFar.add(tree);

//     for(Node child : tree.children) {
//       List<Node> curPath = dfs(child, pathSoFar, xtarget);
//       if (curPath != null)
//         return curPath;
//     }

//     return null;
//   }
// }
