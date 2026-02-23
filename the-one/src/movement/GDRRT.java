package movement;
import core.Coord;
import java.util.*;

class Node {
  Set<Node> children;
  Node parent;
  Coord position;
  double cost;
  
  Node (Coord c, double cost) {
    this.position = c.clone();
    this.cost = cost;
    this.parent = null;
    this.children = new HashSet<Node>();
  }

  Node (Coord c, double cost, Node parent) {
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
}

public class GDRRT {
  // tree for storing the current path options
  private static Node tree;
  
  // size of the sampling sphere
  private static double d;
  private static Coord posTemp;

  // maximum extension distance
  private static double delta;
  
  // TODO: make these proportional
  private static final int BASE_EXTENSION = 10;
  private static final int MIN_EXTENSION = 1;
  private static double globalMinDistance;
  private static double nearRadius = 5;

  private static final int OBSTACLE_CONSTANT = 1;

  // bounds for the solution space (hard coded for now, calculate it later)
  private static Coord topLeft = new Coord(100, 100);
  private static Coord bottomRight = new Coord(1000, 1000);
  
  // total no of iterations algorithm runs far
  // higher means better path
  private static int totalIterations = 10;


  
  public static Path findPath(MapBasedMovement mapBasedMM, Coord startLoc, Coord endLoc) {
    tree = new Node(startLoc, 0);
    posTemp = tree.getLocation();
    Coord rand = sample(d, posTemp);
    double minDistanceFromGoal = startLoc.distance(endLoc);
    // TODO: set this proportional to the overall distance
    globalMinDistance = startLoc.distance(endLoc);

    for (int i = 0; i < totalIterations; i++) {
      Coord nearest = findNearest(tree, rand);
      Coord newNode = steer(nearest, rand, delta);
      
      // TODO: define obstacles from mapBasedMM
      // not sure what type obstacles should have or if there should be multiple arguments
      if (collisionFree(newNode, obstacles)) {
        delta = BASE_EXTENSION;
        double currentDistance = distance(newNode, endLoc);
        if (currentDistance < minDistanceFromGoal) { 
          posTemp = newNode;
          minDistanceFromGoal = currentDistance;
        }

        // what is this globalMinDistance exactly?
        if (minDistanceFromGoal < globalMinDistance) {
          delta = MIN_EXTENSION;
        }

        rand = sample(d, posTemp);
        List<Node> nearNodes = near(tree, newNode,nearRadius);
        Node minNode = chooseParent(nearNodes, nearest, newNode);
        insertNode(minNode, newNode);
        tree = rewire(tree, nearNodes, minNode, newNode);
      } else {
        delta = delta + OBSTACLE_CONSTANT;
        rand = sample();
      }

      return generatePath(tree);
    }
  }

  /*
   * Generates a random coord bounded by topLeft & bottomRight Coord object
   */
  private static Coord sample() {
    // get bounds from topLeft & bottomRight
    double leftX = topLeft.getX();
    double topY = topLeft.getY();
    double rightX = bottomRight.getX();
    double bottomY = bottomRight.getY();

    // generate a random (x, y) coordinate within bounds
    Random randomInstance = new Random();
    double x = leftX + (rightX - leftX) * randomInstance.nextDouble();
    double y = topY + (bottomY - topY) * randomInstance.nextDouble();

    return new Coord(x, y);
  }
  
  /*
   * Generates a random coord bounded by topLeft & bottomRight Coord object
   * The sample space is further limited by the hypersphere around posTemp
   */
  private static Coord sample(double d, Coord posTemp) {
    // get bounds from topLeft,  bottomRight & posTemp + d
    double leftX = Math.max(topLeft.getX(), posTemp.getX() - d);
    double topY = Math.max(topLeft.getY(), posTemp.getY() - d);
    double rightX = Math.min(bottomRight.getX(), posTemp.getX() + d);
    double bottomY = Math.min(bottomRight.getY(), posTemp.getY() + d);

    // generate a random (x, y) coordinate within bounds
    Random randomInstance = new Random();
    double x = leftX + (rightX - leftX) * randomInstance.nextDouble();
    double y = topY + (bottomY - topY) * randomInstance.nextDouble();

    return new Coord(x, y);
  }
  
  /*
   * Returns the rand Coord if it is reachable by covering delta distance
   * from the nearest node in the RR Tree.
   * If not, returns the furthest Coord in the direction of rand 
   * that delta distance allows to go to.
   */
  private static Coord steer(Coord nearest, Coord rand, double delta) {
    double distance = nearest.distance(rand);
    if (distance <= delta) return rand;
    
    double xDistance = rand.getX() - nearest.getX();
    double yDistance = rand.getY() - nearest.getY();
    double parts = delta / distance;
    
    double x = nearest.getX() + (parts * xDistance);
    double y = nearest.getY() + (parts * yDistance);

    return new Coord(x, y);
  }
  
  // TODO: implement this
  //Traversing the tree in  Level order  way 
  //and try to check for min distance for each node;
  private static Coord  findNearest(Node tree, Coord rand) {
      Node xNearest = null;
      double distance  = Integer.MAX_VALUE ; 
      if( tree==null) { 
         return null;
      }
      Queue<Node> q = new LinkedList<>();
      q.add(tree);
      while( !q.isEmpty()) {
            Node temp = q.remove();
            double temp_distance = distance(rand, temp.position );
            if(  temp_distance > distance ) { 
                xNearest = temp ;
                distance = temp_distance;                      
            }
            for ( Node i : temp.children ){ 
                  if ( i!=null ) { 
                        q.add(i) ;
                  }

            }
      }

      return xNearest.position;
  }

  /*
   * Return the Node from the tree which have least cost 
    * from itself to the newNode and
    * Doing this for checking there is any cases there is another 
    * Node nearer than nearest 
   */
  private static Node chooseParent(List<Node> nearNodes, Node nearest, Coord newNode){
          Node  min = nearest; 
           min.cost = distance( min.position, newNode)  + min.cost;
          for (int i = 0 ;i<nearNodes.size() ;i++ ) { 
                Node temp = nearNodes.get(i);                
                if ( temp.cost+distance( temp.position, newNode)  >  min.cost)  { 
                      min = temp;
                      min.cost = temp.cost+distance( temp.position, newNode);
                }
          }
          return min ;       
  }

  /// Euclidian distance between two points in the space 
  private static double distance(Coord a , Coord b) {
    return Math.sqrt(Math.pow(b.getX() - a.getX(), 2) + Math.pow(b.getY() - a.getY(), 2));
  }   


  /*
    * Inserting the Node as the child of the parent of 
  */
  public static void insertNode(Node minNode, Node newNode) {
    minNode.children.add(newNode);
  }


  /*
   * Finds all the nodes in the hypersphere with the newNode as center
   * and radius rNear
   */
  static List<Node> near(Node tree, Node newNode, double rNear) {

  }
}
