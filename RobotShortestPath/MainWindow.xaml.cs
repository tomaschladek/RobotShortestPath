using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;
using System.Windows.Data;
using System.Windows.Documents;
using System.Windows.Input;
using System.Windows.Media;
using System.Windows.Media.Imaging;
using System.Windows.Navigation;
using System.Windows.Shapes;

namespace RobotShortestPath
{
    public class StateDto
    {
        private EMode _mode = EMode.None;

        public EMode Mode
        {
            get
            {
                return _mode;
            }
            set
            {
                _mode = value;
            }
        }
    }

    /// <summary>
    /// Interaction logic for MainWindow.xaml
    /// </summary>
    public partial class MainWindow : Window
    {
        private Point _start;
        private Point _end;
        private IList<Point> _obstacle;
        private IList<IList<Point>> _obstacles = new List<IList<Point>>();

        public MainWindow()
        {
            InitializeComponent();
            this.Loaded += OnLoaded;
            DataContext = Mode = new StateDto();
        }

        public StateDto Mode { get; set; }

        private void OnLoaded(object sender, RoutedEventArgs e)
        {
            DataContext = Mode;

            DrawMesh();
            var start = new Point(30, 100);
            var end = new Point(450, 370);

            var obstacles = new List<IList<Point>>
            {
                new List<Point>
                {
                    new Point(300, 300),
                    new Point(500, 300),
                    new Point(300, 500),
                },

                new List<Point>
                {
                    new Point(270, 270),
                    new Point(300, 100),
                    new Point(100, 300),
                },

                new List<Point>
                {
                    new Point(150, 500),
                    new Point(100, 350),
                    new Point(150, 300),
                },

                new List<Point>
                {
                    new Point(500, 150),
                    new Point(300, 150),
                    new Point(300, 300),
                    new Point(500, 300),
                },
            };

            //DrawExample(start, end, obstacles);
        }

        private void DrawExample(Point start, Point end, IList<IList<Point>> obstacles)
        {
            DrawSetup(start, end, obstacles);
            var graph = GetFullGraph(start, end, obstacles);
            var reducedGraph = GetReducedGraph(obstacles, graph);
            DrawGraph(reducedGraph, Brushes.Blue);
            var path = GetPath(start, end, reducedGraph) ?? new List<Point>();
            path.Add(start);
            for (int index = 1; index < path.Count; index++)
            {
                DrawLine(path[index - 1], path[index], Brushes.Orange);
            }
        }

        private List<Point> GetPath(Point start, Point end, Dictionary<Point, IList<Point>> graph)
        {
            var g = new DijkstraAlgorithm();
            foreach (var node in graph)
            {
                var edges = new Dictionary<Point, double>();
                foreach (var neighbour in node.Value)
                {
                    var distance = Math.Sqrt(Math.Pow(node.Key.X-neighbour.X,2) + Math.Pow(node.Key.Y-neighbour.Y,2));
                    if (!edges.ContainsKey(neighbour))
                    {
                        edges.Add(neighbour, distance);
                    }
                    else if (edges[neighbour] > distance)
                    {
                        edges[neighbour] = distance;
                    }
                }
                g.add_vertex(node.Key, edges);
            }

            return g.shortest_path(start, end);
        }

        private Dictionary<Point, IList<Point>> GetReducedGraph(IList<IList<Point>> obstacles, Dictionary<Point, IList<Point>> graph)
        {
            var reducedGraph = new Dictionary<Point, IList<Point>>();
            foreach (var node in graph)
            {
                foreach (var pointTo in node.Value)
                {
                    if (!reducedGraph.ContainsKey(node.Key))
                    {
                        reducedGraph.Add(node.Key, new List<Point>());
                    }

                    if (IsLineIntersectingAnyObstacle(obstacles, node.Key, pointTo))
                    {
                        reducedGraph[node.Key].Add(pointTo);
                    }


                }
            }
            AddObstaclesEdges(obstacles, reducedGraph);

            return reducedGraph;
        }

        private static bool IsLineIntersectingAnyObstacle(IList<IList<Point>> obstacles, Point pointFrom, Point pointTo)
        {
            return obstacles.All(obstacle =>
            {
                for (int index = 1; index < obstacle.Count; index++)
                {
                    Vector intersection;
                    if (LineSegementsIntersect(pointFrom, pointTo, obstacle[index - 1], obstacle[index], out intersection))
                    {
                        return false;
                    }

                    //for (int offsetIndex = 0; offsetIndex < obstacle.Count; offsetIndex++)
                    //{
                    //    for (int offsetInnerIndex = 0; offsetInnerIndex < obstacle.Count - 1; offsetInnerIndex++)
                    //    {
                    //        var otherNode = (offsetIndex + 1 + offsetInnerIndex)%obstacle.Count;
                    //        if (LineSegementsIntersect(pointFrom, pointTo, obstacle[offsetIndex], obstacle[
                    //                otherNode],
                    //            out intersection))
                    //        {
                    //            return false;
                    //        }
                    //    }
                    //}
                }

                Vector intersection2;
                return !LineSegementsIntersect(pointFrom, pointTo, obstacle[0], obstacle[obstacle.Count - 1], out intersection2);
            });
        }

        private static void AddObstaclesEdges(IList<IList<Point>> obstacles, Dictionary<Point, IList<Point>> reducedGraph)
        {
            foreach (var obstacle in obstacles)
            {
                for (int index = 0; index < obstacle.Count; index++)
                {
                    if (!reducedGraph.ContainsKey(obstacle[index]))
                    {
                        reducedGraph.Add(obstacle[index], new List<Point>());
                    }

                    var followingIndex = (index + 1) % obstacle.Count;
                    //if (!IsLineIntersectingAnyObstacle(obstacles, obstacle[index],obstacle[followingIndex]))
                    //{
                        reducedGraph[obstacle[index]].Add(obstacle[followingIndex]);
                    //}

                    var previousIndex = (index - 1 + obstacle.Count) % obstacle.Count;
                    //if (!IsLineIntersectingAnyObstacle(obstacles, obstacle[index], obstacle[previousIndex]))
                    {
                        reducedGraph[obstacle[index]].Add(obstacle[previousIndex]);
                    }
                }
            }
        }

        private void DrawGraph(Dictionary<Point, IList<Point>> graph, Brush brush)
        {
            foreach (var pair in graph)
            {
                foreach (var pointTo in pair.Value)
                {
                    DrawLine(pair.Key, pointTo, brush);
                }
            }
        }

        private static Dictionary<Point, IList<Point>> GetFullGraph(Point start, Point end, IList<IList<Point>> obstacles)
        {
            var allCollections = new List<IList<Point>>(obstacles);
            allCollections.Add(new List<Point> { start });
            allCollections.Add(new List<Point> { end });

            var graph = new Dictionary<Point, IList<Point>>();

            for (int fromIndex = 0; fromIndex < allCollections.Count; fromIndex++)
            {
                for (int innerIndex = fromIndex + 1; innerIndex < allCollections.Count; innerIndex++)
                {
                    foreach (var fromCollection in allCollections[fromIndex])
                    {
                        if (!graph.ContainsKey(fromCollection))
                        {
                            graph.Add(fromCollection, new List<Point>());
                        }

                        foreach (var toCollection in allCollections[innerIndex])
                        {
                            if (!graph.ContainsKey(toCollection))
                            {
                                graph.Add(toCollection, new List<Point>());
                            }
                            graph[fromCollection].Add(toCollection);
                            graph[toCollection].Add(fromCollection);
                        }
                    }
                }
            }

            return graph;
        }

        private void DrawLine(Point from, Point to, Brush brush, int thickness = 1)
        {
            var line = new Line
            {
                Stroke = brush,
                X1 = from.X,
                X2 = to.X,
                Y1 = from.Y,
                Y2 = to.Y,
                StrokeThickness = thickness
            };

            AreaCanvas.Children.Add(line);
        }

        private void DrawSetup(Point start, Point end, IList<IList<Point>> obstacles)
        {
            foreach (var points in obstacles)
            {
                DrawObstacle(points);
            }

            DrawPoint(start, Colors.Green);
            DrawPoint(end, Colors.Red);
        }

        private void DrawObstacle(IList<Point> points)
        {
            var myPolygon = new Polygon
            {
                Fill = Brushes.Gray,
                Points = new PointCollection(points)
            };
            AreaCanvas.Children.Add(myPolygon);

        }

        private void DrawPoint(Point point, Color color)
        {
            var size = 20;
            Ellipse myEllipse = new Ellipse
            {
                Fill = new SolidColorBrush(color),
                StrokeThickness = 2,
                Stroke = Brushes.Transparent,
                Width = size,
                Height = size
            };

            AreaCanvas.Children.Add(myEllipse);

            Canvas.SetLeft(myEllipse,point.X - size / 2);
            Canvas.SetTop(myEllipse,point.Y - size / 2);
        }

        private void DrawMesh()
        {
            var step = 20;
            var brush = Brushes.LightSteelBlue;
            var brush100 = Brushes.CornflowerBlue;
            var thickness = 1;
            for (int left = 0; left < AreaCanvas.ActualWidth; left += step)
            {
                DrawLine(new Point(left,0),new Point(left,AreaCanvas.ActualHeight),brush, thickness);
            }

            for (int left = 0; left < AreaCanvas.ActualHeight; left += step)
            {
                DrawLine(new Point(0,left), new Point(AreaCanvas.ActualWidth, left), brush, thickness);
            }

            for (int left = 0; left < AreaCanvas.ActualWidth; left += step*5)
            {
                DrawLine(new Point(left,0),new Point(left,AreaCanvas.ActualHeight),brush100, thickness);
            }

            for (int left = 0; left < AreaCanvas.ActualHeight; left += step*5)
            {
                DrawLine(new Point(0,left), new Point(AreaCanvas.ActualWidth, left), brush100, thickness);
            }
        }

        public static bool LineSegementsIntersect(Point point, Point point2, Point qoint, Point qoint2,
            out Vector intersection, bool considerCollinearOverlapAsIntersect = false)
        {
            intersection = new Vector();

            var p = new Vector(point.X, point.Y);
            var p2 = new Vector(point2.X, point2.Y);
            var q = new Vector(qoint.X, qoint.Y);
            var q2 = new Vector(qoint2.X, qoint2.Y);

            var r = p2 - p;
            var s = q2 - q;
            var rxs = r.Cross(s);
            var qpxr = (q - p).Cross(r);

            // If r x s = 0 and (q - p) x r = 0, then the two lines are collinear.
            if (rxs.IsZero() && qpxr.IsZero())
            {
                // 1. If either  0 <= (q - p) * r <= r * r or 0 <= (p - q) * s <= * s
                // then the two lines are overlapping,
                if (considerCollinearOverlapAsIntersect)
                    if ((0 <= (q - p) * r && (q - p) * r <= r * r) || (0 <= (p - q) * s && (p - q) * s <= s * s))
                        return true;

                // 2. If neither 0 <= (q - p) * r = r * r nor 0 <= (p - q) * s <= s * s
                // then the two lines are collinear but disjoint.
                // No need to implement this expression, as it follows from the expression above.
                return false;
            }

            // 3. If r x s = 0 and (q - p) x r != 0, then the two lines are parallel and non-intersecting.
            if (rxs.IsZero() && !qpxr.IsZero())
                return false;

            // t = (q - p) x s / (r x s)
            var t = (q - p).Cross(s) / rxs;

            // u = (q - p) x r / (r x s)

            var u = (q - p).Cross(r) / rxs;

            // 4. If r x s != 0 and 0 <= t <= 1 and 0 <= u <= 1
            // the two line segments meet at the point p + t r = q + u s.
            if (!rxs.IsZero() && (0 <= t && t <= 1) && (0 <= u && u <= 1))
            {
                // We can calculate the intersection point using either t or u.
                intersection = p + t * r;
                if (Equals(intersection, p) || Equals(intersection, p2))
                    return false;
                // An intersection was found.
                return true;
            }

            // 5. Otherwise, the two line segments are not parallel but do not intersect.
            return false;
        }

        private void SetStart(object sender, RoutedEventArgs e)
        {
            Mode.Mode = EMode.Start;
        }

        private void SetEnd(object sender, RoutedEventArgs e)
        {
            Mode.Mode = EMode.End;
        }

        private void StartObstacle(object sender, RoutedEventArgs e)
        {
            Mode.Mode = EMode.Obstacle;
            _obstacle = new List<Point>();
        }

        private void AreaCanvas_OnMouseLeftButtonDown(object sender, MouseButtonEventArgs e)
        {
            switch (Mode.Mode)
            {
                case EMode.Start:
                    if (_start != default(Point))
                    {
                        return;
                    }
                    _start = GetPoint();
                    DrawPoint(_start,Colors.Green);
                    Mode.Mode = EMode.None;
                    break;
                case EMode.End:
                    if (_end != default(Point))
                    {
                        return;
                    }
                    _end = GetPoint();
                    DrawPoint(_end, Colors.Red);
                    Mode.Mode = EMode.None;
                    break;
                case EMode.Obstacle:
                    var point = GetPoint();
                    _obstacle.Add(point);
                    DrawPoint(point, Colors.Gray);
                    break;
            }
        }

        private Point GetPoint()
        {
            return Mouse.GetPosition(AreaCanvas);
        }

        private void EndObstacle(object sender, RoutedEventArgs e)
        {
            if (_obstacle != null)
            {
                _obstacles.Add(_obstacle);
                DrawObstacle(_obstacle);
                _obstacle = null;
            }
            Mode.Mode = EMode.None;
        }

        private void DrawFull(object sender, RoutedEventArgs e)
        {
            var graph = GetFullGraph(_start, _end, _obstacles);
            DrawGraph(graph, Brushes.LightSkyBlue);
        }

        private void DrawReduced(object sender, RoutedEventArgs e)
        {
            var graph = GetFullGraph(_start, _end, _obstacles);
            var reducedGraph = GetReducedGraph(_obstacles, graph);
            DrawGraph(reducedGraph, Brushes.Blue);
        }

        private void DrawPath(object sender, RoutedEventArgs e)
        {
            var graph = GetFullGraph(_start, _end, _obstacles);
            var reducedGraph = GetReducedGraph(_obstacles, graph);
            var path = GetPath(_start, _end, reducedGraph) ?? new List<Point>();
            path.Add(_start);
            for (int index = 1; index < path.Count; index++)
            {
                DrawLine(path[index - 1], path[index], Brushes.Orange,3);
            }
        }

        private void DrawResult(object sender, RoutedEventArgs e)
        {
            var graph = GetFullGraph(_start, _end, _obstacles);
            var reducedGraph = GetReducedGraph(_obstacles, graph);
            DrawGraph(reducedGraph, Brushes.Blue);
            var path = GetPath(_start, _end, reducedGraph) ?? new List<Point>();
            path.Add(_start);
            for (int index = 1; index < path.Count; index++)
            {
                DrawLine(path[index - 1], path[index], Brushes.Orange);
            }
        }

        private void Clear(object sender, RoutedEventArgs e)
        {
            _start = default(Point);
            _end = default(Point);
            _obstacles = new List<IList<Point>>();
            Mode.Mode = EMode.None;
            AreaCanvas.Children.Clear();
            DrawMesh();
        }

        private void RemoveEdges(object sender, RoutedEventArgs e)
        {
            AreaCanvas.Children.Clear();
            DrawMesh();
            DrawSetup(_start,_end,_obstacles);
        }
    }
    public class Vector
    {
        public double X;
        public double Y;

        // Constructors.
        public Vector(double x, double y) { X = x; Y = y; }
        public Vector() : this(double.NaN, double.NaN) { }

        public static Vector operator -(Vector v, Vector w)
        {
            return new Vector(v.X - w.X, v.Y - w.Y);
        }

        public static Vector operator +(Vector v, Vector w)
        {
            return new Vector(v.X + w.X, v.Y + w.Y);
        }

        public static double operator *(Vector v, Vector w)
        {
            return v.X * w.X + v.Y * w.Y;
        }

        public static Vector operator *(Vector v, double mult)
        {
            return new Vector(v.X * mult, v.Y * mult);
        }

        public static Vector operator *(double mult, Vector v)
        {
            return new Vector(v.X * mult, v.Y * mult);
        }

        public double Cross(Vector v)
        {
            return X * v.Y - Y * v.X;
        }

        public override bool Equals(object obj)
        {
            var v = (Vector)obj;
            return (X - v.X).IsZero() && (Y - v.Y).IsZero();
        }
    }

    public static class Extensions
    {
        private const double Epsilon = 1e-10;

        public static bool IsZero(this double d)
        {
            return Math.Abs(d) < Epsilon;
        }
    }

    public enum EMode
    {
        None,Start,End,Obstacle
    }
}
