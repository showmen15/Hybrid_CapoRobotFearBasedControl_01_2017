package pl.edu.agh.capo.fear.collisions;

import com.vividsolutions.jts.math.Vector2D;

import pl.edu.agh.capo.fear.data.Trajectory;

public interface CollisionDetector
{

	public int getCollidingTrajecotryMinStepNumber(Trajectory trajectory);

	// public int getCollidingTrajecotryMaxStepNumber(Trajectory trajectory);

	// public Vector2D getNextDestination(Vector2D currentLocation, Vector2D
	// targetDestination);
}
