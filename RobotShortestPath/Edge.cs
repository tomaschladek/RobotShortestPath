using System.Collections.Generic;
using System.Windows;

namespace RobotShortestPath
{
    public class Node
    {
        public Point Position { get; set; }
        public IList<Point> Neighbours { get; set; }

        public Node(Point position, IList<Point> neighbours)
        {
            Position = position;
            Neighbours = neighbours;
        }
    }
}