using System;
using System.Collections.Generic;
using System.Configuration;
using System.Data;
using System.Linq;
using System.Threading.Tasks;
using System.Windows;
using System.Windows.Controls;

namespace RobotShortestPath
{
    /// <summary>
    /// Interaction logic for App.xaml
    /// </summary>
    public partial class App : Application
    {
        protected override void OnStartup(StartupEventArgs e)
        {
            //Example();
        }

        private static void Example()
        {
            //var g = new DijkstraAlgorithm();
            //g.add_vertex('A', new Dictionary<long, int>() { { 'B', 7 }, { 'C', 8 } });
            //g.add_vertex('B', new Dictionary<long, int>() { { 'A', 7 }, { 'F', 2 } });
            //g.add_vertex('C', new Dictionary<long, int>() { { 'A', 8 }, { 'F', 6 }, { 'G', 4 } });
            //g.add_vertex('D', new Dictionary<long, int>() { { 'F', 8 } });
            //g.add_vertex('E', new Dictionary<long, int>() { { 'H', 1 } });
            //g.add_vertex('F', new Dictionary<long, int>() { { 'B', 2 }, { 'C', 6 }, { 'D', 8 }, { 'G', 9 }, { 'H', 3 } });
            //g.add_vertex('G', new Dictionary<long, int>() { { 'C', 4 }, { 'F', 9 } });
            //g.add_vertex('H', new Dictionary<long, int>() { { 'E', 1 }, { 'F', 3 } });

            //g.shortest_path('A', 'H').ForEach(x => Console.WriteLine(x));
        }
    }
}
