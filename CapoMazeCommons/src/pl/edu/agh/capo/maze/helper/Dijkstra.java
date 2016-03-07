package pl.edu.agh.capo.maze.helper;

import java.util.*;

public class Dijkstra
{
	private Hashtable<String, Hashtable<String, Double>> vertices = new Hashtable<String, Hashtable<String, Double>>();

	public void add_vertex(String name, String key_edges, Double value_edges)
	{
		if (!vertices.containsKey(name))
			vertices.put(name, new Hashtable<String, Double>());

		vertices.get(name).put(key_edges, value_edges);
	}

	public List<String> shortest_path(String start, String finish)
	{
		Hashtable<String, String> previous = new Hashtable<String, String>();
		final Hashtable<String, Double> distances = new Hashtable<String, Double>();
		List<String> nodes = new ArrayList<String>();

		List<String> path = null;

		Enumeration<String> verticesEnumKey = vertices.keys();

		while (verticesEnumKey.hasMoreElements())
		{
			String vertexKey = verticesEnumKey.nextElement();

			if (vertexKey.equals(start))
			{
				distances.put(vertexKey, 0.0);
			}
			else
			{
				distances.put(vertexKey, Double.MAX_VALUE);
			}

			nodes.add(vertexKey);
		}

		while (nodes.size() != 0)
		{
			Collections.sort(nodes, new Comparator<String>()
			{
				public int compare(String x, String y)
				{

					if (distances.get(x) == distances.get(y))
						return 0;
					else
						if (distances.get(x) > distances.get(y))
							return 1;
						else
							return -1;
				}
			});

			String smallest = nodes.get(0);
			nodes.remove(smallest);

			if (smallest.equals(finish))
			{
				path = new ArrayList<String>();

				while (previous.containsKey(smallest))
				{
					path.add(smallest);
					smallest = previous.get(smallest);
				}

				break;
			}

			if (distances.get(smallest) == Double.MAX_VALUE)
			{
				break;
			}

			Hashtable<String, Double> neighbor = vertices.get(smallest);
			Enumeration<String> neighborEnumKey = neighbor.keys();

			while (neighborEnumKey.hasMoreElements())
			{
				String neighborKey = neighborEnumKey.nextElement();
				Double neighborValue = neighbor.get(neighborKey);

				Double alt = distances.get(smallest) + neighborValue;
				if (alt < distances.get(neighborKey))
				{
					distances.put(neighborKey, alt);
					previous.put(neighborKey, smallest);
				}
			}
		}

		return path;
	}

	private double getValueForEdge(String verticFrom, String verticTo)
	{
		return vertices.get(verticFrom).get(verticTo);
	}

	public Dijkstra()
	{

	}
}
