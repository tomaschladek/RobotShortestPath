using System;
using System.Collections.Generic;
using System.Windows;

namespace RobotShortestPath
{
    public class DijkstraAlgorithm
    {
        Dictionary<Point, Dictionary<Point, double>> vertices = new Dictionary<Point, Dictionary<Point, double>>();

        public void add_vertex(Point name, Dictionary<Point, double> edges)
        {
            vertices[name] = edges;
        }

        public List<Point> shortest_path(Point start, Point finish)
        {
            var previous = new Dictionary<Point, Point>();
            var distances = new Dictionary<Point, double>();
            var nodes = new List<Point>();

            List<Point> path = null;

            foreach (var vertex in vertices)
            {
                if (vertex.Key == start)
                {
                    distances[vertex.Key] = 0;
                }
                else
                {
                    distances[vertex.Key] = int.MaxValue;
                }

                nodes.Add(vertex.Key);
            }

            while (nodes.Count != 0)
            {
                nodes.Sort((x, y) => (int) (distances[x] - distances[y]));

                var smallest = nodes[0];
                var smallestDistance = distances[nodes[0]];
                nodes.Remove(smallest);

                if (smallest == finish)
                {
                    path = new List<Point>();
                    while (previous.ContainsKey(smallest))
                    {
                        path.Add(smallest);
                        smallest = previous[smallest];
                    }

                    break;
                }

                if (Math.Abs(distances[smallest] - int.MaxValue) < 0.0005)
                {
                    break;
                }

                foreach (var neighbor in vertices[smallest])
                {
                    var alt = distances[smallest] + neighbor.Value;
                    if (alt < distances[neighbor.Key])
                    {
                        distances[neighbor.Key] = alt;
                        previous[neighbor.Key] = smallest;
                    }
                }
            }

            return path;
        }
    }
}