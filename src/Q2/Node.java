package Q2;

import java.util.Objects;

/**
 * Created by Kevin on 03/23/2017.
 */
public class Node implements Comparable<Node> {
    private int g; // cost from start node, to current node
    private int h; // ideal cost from current node, to goal node
    private Coordinate coordinate;
    private Node parent;
    private char type; // obstacle, empty, path, start, or goal

    public Node(char _type, int _x, int _y) {
        type = _type;
        coordinate = new Coordinate(_x, _y);
        h = Integer.MAX_VALUE;
    }

    /**
     * Constructor used to copy a node
     * @param another
     */
    public Node(Node another) {
        this.g = another.g;
        this.h = another.h;
        this.coordinate = another.coordinate;
        this.parent = another.parent;
        this.type = another.type;
    }

    // setters
    public void setG(int newG) {
        g = newG;
    }

    public void setH(int newH) {
        h = newH;
    }

    public void setParent(Node _parent) {
        parent = _parent;
    }

    public void setType(char _type){
        type = _type;
    }

    //getters
    public int getG() {
        return g;
    }

    public int getH() {
        return h;
    }

    public int getF() {
        return g + h;
    }

    public Node getParent() {
        return parent;
    }

    public Coordinate getCoordinate() {
        return coordinate;
    }

    public char getType() {
        return type;
    }

    public String toString() {
        return coordinate + "; G:" + g + ", H:" + h + ", F:" + getF() + "; Type: " + type;
    }

    @Override
    public int compareTo(Node n) {
        return 0;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof Node)) {
            return false;
        }
        Node other = (Node) obj;
        return this.coordinate == other.getCoordinate();
    }

    @Override
    public int hashCode() {
        return Objects.hash(coordinate, type);
    }
}
