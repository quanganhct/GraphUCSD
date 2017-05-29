package roadgraph;

import geography.GeographicPoint;

public class Node {
	private STATE state = STATE.NONE;
	private GeographicPoint point;

	public Node(GeographicPoint p) {
		this.point = p;
	}

	@Override
	public boolean equals(Object obj) {
		// TODO Auto-generated method stub
		if (!(obj instanceof Node) || obj == null) {
			return false;
		}
		Node node = (Node) obj;
		if (this == node) {
			return true;
		} else if (this.point == node.point) {
			return true;
		} else {
			return this.point.x == node.point.x && this.point.y == node.point.y;
		}
	}

	public GeographicPoint getPoint() {
		return point;
	}

	public STATE getState() {
		return state;
	}

	public void setState(STATE s) {
		this.state = s;
	}
}
