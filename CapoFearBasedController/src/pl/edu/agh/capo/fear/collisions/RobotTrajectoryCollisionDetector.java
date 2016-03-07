package pl.edu.agh.capo.fear.collisions;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;

import pl.edu.agh.capo.fear.CapoSafeTrajectoryGenerator;
import pl.edu.agh.capo.fear.communication.StateMessageConsumer;
import pl.edu.agh.capo.fear.data.Location;
import pl.edu.agh.capo.fear.data.LocationInTime;
import pl.edu.agh.capo.fear.data.Trajectory;
import pl.edu.agh.capo.robot.CapoRobotMotionModel;

public class RobotTrajectoryCollisionDetector implements CollisionDetector, StateMessageConsumer {

	protected HashMap<Integer, Trajectory> robotsTrajectories = new HashMap<Integer, Trajectory>();
	protected int skippedRobotId;
	
	public RobotTrajectoryCollisionDetector(int skippedRobotId)
	{
		this.skippedRobotId = skippedRobotId;
	}
	
    @Override
	public int getCollidingTrajecotryMinStepNumber(Trajectory trajectory) 
    {
    	ArrayList<Iterator<LocationInTime>> otherTrajectoriesLocaiontsIterators = new ArrayList<Iterator<LocationInTime>>();
    	synchronized (robotsTrajectories) 
    	{
	    	for (Trajectory otherTrajectory : robotsTrajectories.values())
	    	{
	    		Iterator<LocationInTime> otherTrajectoryLocaiontsIterator = otherTrajectory.getTrajectorySteps().iterator();
	    		if (otherTrajectoryLocaiontsIterator.hasNext())
	    			otherTrajectoryLocaiontsIterator.next();		// set at second item
	    		if (otherTrajectoryLocaiontsIterator.hasNext())	    			
	    			otherTrajectoriesLocaiontsIterators.add(otherTrajectoryLocaiontsIterator);
	    	}
    	}
    	int stepId = 1;
    	Iterator<LocationInTime> thisTrajectoryLocaiontsIterators = trajectory.getTrajectorySteps().iterator();
    	if (!thisTrajectoryLocaiontsIterators.hasNext())
    		return -1;
    	thisTrajectoryLocaiontsIterators.next();    	
    	while (thisTrajectoryLocaiontsIterators.hasNext())
    	{
    		Location thisTrajectoryLocation = thisTrajectoryLocaiontsIterators.next().getLocation();
    		for (Iterator<LocationInTime> otherTrajectoryLocaiontsIterator : otherTrajectoriesLocaiontsIterators)
    		{
    			if (otherTrajectoryLocaiontsIterator.hasNext())	
    			{
    				Location otherTrajectoryLocation = otherTrajectoryLocaiontsIterator.next().getLocation();
    				if (thisTrajectoryLocation.getDistance(otherTrajectoryLocation) < CapoRobotMotionModel.robotDiameter)
    					return stepId;
    			}
    		}    		
    		stepId++;
    	}
		return -1;
	}

	@Override
	public void consumeMessage(Trajectory trajectory) 
	{
    	synchronized (robotsTrajectories) 
    	{		
    		if (trajectory.getRobotId() != skippedRobotId)
    			robotsTrajectories.put(trajectory.getRobotId(), trajectory);		
    	}
	}

}
