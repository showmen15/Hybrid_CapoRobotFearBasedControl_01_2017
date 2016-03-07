package pl.edu.agh.capo.maze;

public class Gate
{
	private String type;

	private String id;

	private String kind;

	private double blocked;

	private Coordinates from;

	private Coordinates to;

	private Coordinates center;

	public Coordinates getTo()
	{
		return to;
	}

	public void setTo(Coordinates to)
	{
		this.to = to;
	}

	public String getId()
	{
		return id;
	}

	public void setId(String id)
	{
		this.id = id;
	}

	public String getKind()
	{
		return kind;
	}

	public void setKind(String kind)
	{
		this.kind = kind;
	}

	public double getBlocked()
	{
		return blocked;
	}

	public void setBlocked(double blocked)
	{
		this.blocked = blocked;
	}

	public Coordinates getFrom()
	{
		return from;
	}

	public void setFrom(Coordinates from)
	{
		this.from = from;
	}

	public Coordinates getCenter()
	{
		return center;
	}

	public void setCenter(Coordinates center)
	{
		this.center = center;
	}
}
