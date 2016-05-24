package pl.edu.agh.capo.fear.data;

import java.io.Serializable;
import java.util.ArrayList;

import com.vividsolutions.jts.math.Vector2D;

public class Trajectory implements Serializable
{
	private static final long serialVersionUID = -8494941241191154511L;
	protected ArrayList<LocationInTime> trajectorySteps;
	protected Location targetLocation;
	protected Location firstStepLocation, lastStepLocation;
	protected double fearFactor;

	public Location getLastStepLocation()
	{
		return lastStepLocation;
	}

	public Location getFirstStepLocation()
	{
		return firstStepLocation;
	}

	protected int robotId;

	public Trajectory(Vector2D targetLocation)
	{
		super();
		this.targetLocation = new Location(targetLocation.getX(), targetLocation.getY(), 0);
		this.trajectorySteps = new ArrayList<LocationInTime>();
	}

	public void addTrajectoryStep(LocationInTime locationInTime)
	{
		if (this.trajectorySteps.size() == 0)
			this.firstStepLocation = locationInTime.location;
		this.trajectorySteps.add(locationInTime);
		this.lastStepLocation = locationInTime.location;
	}

	public Iterable<LocationInTime> getTrajectorySteps()
	{
		return this.trajectorySteps;
	}

	public LocationInTime getLocationInTime(int index)
	{
		return this.trajectorySteps.get(index);
	}

	public Location getTargetLocation()
	{
		return targetLocation;
	}

	public int getRobotId()
	{
		return robotId;
	}

	public void setRobotId(int robotId)
	{
		this.robotId = robotId;
	}

	public double getFearFactor()
	{
		return fearFactor;
	}

	public void setFearFactor(double FearFactor)
	{
		this.fearFactor = FearFactor;
	}
}
