package roadgraph;

/**
 * 
 * @author AnhNQ17
 *
 */
public class Edge {
	private Node endPoint;
	private Node startPoint;
	private String streetName;
	private String streetType;
	private double length;

	public Edge(Node start, Node end, String name, String type, double length) {
		this.startPoint = start;
		this.endPoint = end;
		this.streetName = name;
		this.streetType = type;
		this.length = length;
	}

	public Node getStartPoint() {
		return startPoint;
	}

	public void setStartPoint(Node startPoint) {
		this.startPoint = startPoint;
	}

	public Node getEndPoint() {
		return endPoint;
	}

	public void setEndPoint(Node endPoint) {
		this.endPoint = endPoint;
	}

	public String getStreetName() {
		return streetName;
	}

	public void setStreetName(String streetName) {
		this.streetName = streetName;
	}

	public String getStreetType() {
		return streetType;
	}

	public void setStreetType(String streetType) {
		this.streetType = streetType;
	}

	public double getLength() {
		return length;
	}

	public void setLength(double length) {
		this.length = length;
	}
}
