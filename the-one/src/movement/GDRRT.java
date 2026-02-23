package movement;
import core.Coord;
import java.util.*;

class Node {
  List<Node> children;
  Node parent;
  Coord position;
  double cost;
  
  Node (Coord c, double cost) {
    this.position = c.clone();
    this.cost = cost;
    this.parent = null;
  }

  Node (Coord c, double cost, Node parent) {
    this.position = c.clone();
    this.cost = cost;
    this.parent = parent;
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
  private static double deltaMin;
  private static final int BASE_EXTENSION = 10;
  private static final int MIN_EXTENSION = 1;

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
  private static findNearest(Node tree, Coord rand) {}
}
