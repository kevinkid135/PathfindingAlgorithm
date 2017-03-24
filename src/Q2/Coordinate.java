package Q2;

import java.util.Objects;

/**
 * Created by Kevin on 03/23/2017.
 */
public class Coordinate {
    int xPos, yPos;

    public Coordinate(int _x, int _y) {
        this.xPos = _x;
        this.yPos = _y;
    }

    public String toString() {
        return "xPos: " + xPos + ", yPos: " + yPos;
    }

    @Override
    public boolean equals(Object obj) {
        if (obj == this) {
            return true;
        }
        if (!(obj instanceof Coordinate)) {
            return false;
        }
        Coordinate other = (Coordinate) obj;
        return this.xPos == other.xPos && this.yPos == other.yPos;
    }

    @Override
    public int hashCode() {
        return Objects.hash(xPos, yPos);
    }
}
