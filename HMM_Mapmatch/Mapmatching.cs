using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;
using System.Diagnostics;

namespace HMM_Mapmatch
{
    class Mapmatching
    {
        public int _trj_size = 0;
        private Trajectory _trj;
        private MBR _ext;
        PForest F;
        public List<Edge> complete_route = new List<Edge>();
        //public List<Road_Time> complete_road_t = new List<Road_Time>();
        
        public double tran_time = 0;
        public double emis_time = 0;
        public double viterbi_time = 0;
        public double short_time = 0;
        public int leastNum = 5;
        Graph m_graph;

        private double _distance_limit = 2500; //if distance exceeds 800, stop search

        public Mapmatching()
        { }

        public Mapmatching(Graph g,PForest f)
        {
            m_graph = g;
            this.F = f;
        }

        public Mapmatching(Graph g,int top_k)
        {
            m_graph = g;
            this.leastNum = top_k;
        }

        public void loadTrjfile(string trjfile,double Md)
        {
            double time_interval = 0;//时间间隔
            FileStream fs = new FileStream(trjfile, FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);
            int i = 0;

            if (_trj == null)
                _trj = new Trajectory();
            else
                _trj.Clear();

            if (_ext == null)
                _ext = new MBR();
            else
                _ext.reset();

            //mode
            string line = sr.ReadLine();

            while (line != null)
            {
                if (line == "X\tY\tT\tS\tD")
                {
                    line = sr.ReadLine();
                    continue;
                }    
                string[] items = line.Split(new char[] { '\t' });

                double x = double.Parse(items[0]);
                double y = double.Parse(items[1]);
                //DateTime.Parse(items[0])
                DateTime t = Convert.ToDateTime(items[2]);
                double speed = double.Parse(items[3]);
                double direction = double.Parse(items[4]);
                
                if (i > 0)
                {
                    //以s为时间间隔进行读取
                    TimeSpan ts = t.Subtract(_trj[_trj.Count - 1].T).Duration();
                    if (Convert.ToDouble(ts.TotalSeconds) < time_interval)
                    {
                        line = sr.ReadLine();
                        continue;
                    }
                    if (Math.Abs(x-_trj[_trj.Count - 1].X)<0.00000001 && Math.Abs(y - _trj[_trj.Count - 1].Y)<0.00000001)//剔除重复的点！！！
                    {
                        line = sr.ReadLine();
                        continue;
                    }
                    if (Point.Distance(x, y, _trj[_trj.Count - 1].X, _trj[_trj.Count - 1].Y) < 20)//聚集点除噪音 && long.Parse(items[0]) != 1381604233
                    {
                        line = sr.ReadLine();
                        continue;
                    }

                    

                }
                Point p = new Point(t, x, y,direction,speed);
                _trj.Add(p);
                _ext.unionWith(p);


                if (i > 0)
                {
                    //斜率约束
                    if (i % 5 == 4)
                    {
                        Point p1 = _trj[_trj.Count - 5];
                        Point p2 = _trj[_trj.Count - 4];
                        Point p4 = _trj[_trj.Count - 2];
                        Point p5 = _trj[_trj.Count - 1];

                        double m1 = (p2.Y - p1.Y) / (p2.X - p1.X);
                        double m2 = (p5.Y - p4.Y) / (p5.X - p4.X);

                        if (Math.Abs(m2 - m1) <= Md)
                        {
                            _trj[_trj.Count - 3].X = (p2.X + p4.X) / 2;
                            _trj[_trj.Count - 3].Y = (p2.Y + p4.Y) / 2;
                        }
                    }
                }

                i++;

                line = sr.ReadLine();
            }

            sr.Close();
            fs.Close();

            _trj_size = _trj.Count;
        }

        public void saveMatchedTrjfile(string trjfile,string filename, List<Edge> lists)
        {
            if (!System.IO.Directory.Exists(trjfile))
            {
                System.IO.Directory.CreateDirectory(trjfile);//不存在就创建目录
            }

            FileStream fs = new FileStream(trjfile+"\\"+filename, FileMode.Create, FileAccess.Write);
            StreamWriter sw = new StreamWriter(fs);

            for (int i = 0; i < lists.Count; i++)
            {
                sw.WriteLine(lists[i].ID);
            }

            sw.Close();
            fs.Close();
        }

        public void Mapmatching_HMM(Graph G)
        {
            m_graph = G;
            string patha = @"F:\map matching\frechet dictance\trajectories\traindata\clean.txt";
            Program.writetxt(_trj, patha);

            if (_trj == null || _trj.Count == 0)
            {
                throw new ArgumentOutOfRangeException("trajectory",
                    "Number of points in trajectory should be higher than zero.");
            }

            int edge_num = G.getEdge().Count;

            //the first edge is from edge of the transition, the second edge is the to edge
            Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>> transition_from_to = new Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>>();

            //the first edge is to edge of the transition, the second edge is the from edge
            Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>> transition_to_from = new Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>>();


            //存储相邻GPS点的候选路径之间的路径
            Dictionary<Edge, Dictionary<Edge, List<Edge>>> e1_e2_edges = new Dictionary<Edge, Dictionary<Edge, List<Edge>>>();

            Dictionary<Edge, Dictionary<int, double>> emission = new Dictionary<Edge, Dictionary<int, double>>();
            Dictionary<int, List<Edge>> id_candidate = new Dictionary<int, List<Edge>>();
            Dictionary<Edge, double> initial = new Dictionary<Edge, double>();

            double minmum_radius = 40; // GPS point must be within the 40m of any road segment, or it is regarded as noise
            double maximum_radius = 100;  //we calculate transition probability, for any point, we find at least 3 road segments for candidate
            //if we can not find 3 points within 100 meters, stop searching

            double search_radius = 30;    //we start search within 20 meters, if the start search radius is too large, there are two many edges are selected
            //calculating transition probability will cost a lot of time.

            double step = 10;
            
            double angle_threshod=360;

            //regard points far away from edges as noise, although it may be due to the low accuracy of the road networks
            #region delete points which are far away from road segments
            for (int i = _trj.Count - 1; i >= 0; i--)
            {
                Point p = _trj[i];

                HashSet<Edge> cands_check_good_point = new HashSet<Edge>();
                searchEdgeWithinRadiusOneStep(G._rtree_edge, p, cands_check_good_point, minmum_radius);

                if (cands_check_good_point.Count == 0)
                {
                    _trj.Remove(p);
                }
            }

            if (_trj.Count <= 1) //must have least two points to calculate the transition prob;
                return;

            #endregion

            #region calculate initial probability
            foreach (int id in G.getEdge().Keys)
            {
                Edge e = G.getEdge(id);
                initial.Add(e, 1);
            }
            #endregion
            
            #region calculate emission probability
            for (int i = 0; i < _trj.Count; i++)
            {
                
                HashSet<Edge> cands = new HashSet<Edge>();

                Point p = _trj[i];
                searchEdgeWithMaximumRadiusAngle(G._rtree_edge, p, cands, search_radius, step, leastNum, maximum_radius, angle_threshod);

                foreach (Edge e in cands)
                {

                    Stopwatch st1 = new Stopwatch();
                    st1.Start();//开始计时
                    double measurementProb = MeasurementProbability(p, e, 0,20,0,30);

                    st1.Stop();//终止计时
                    emis_time += st1.ElapsedMilliseconds;

                    if (emission.ContainsKey(e))
                    {
                        Dictionary<int, double> pt_prob = emission[e];
                        if (pt_prob.ContainsKey(i))
                        {
                            pt_prob[i] = measurementProb;
                        }
                        else
                        {
                            pt_prob.Add(i, measurementProb);
                        }
                    }
                    else
                    {
                        Dictionary<int, double> pt_prob = new Dictionary<int, double>();
                        pt_prob.Add(i, measurementProb);
                        emission.Add(e, pt_prob);
                    }

                    //存储每个GPS点搜索到的candidate
                    if (id_candidate.ContainsKey(i))
                    {
                        id_candidate[i].Add(e);
                    }
                    else
                    {
                        List<Edge> candis=new List<Edge>();
                        candis.Add(e);
                        id_candidate.Add(i, candis);
                    }
                }
            }

            #endregion     

            #region calculate transition probability
            //the transition matrix is alway changed with time t (p1.t, p2.t)
            //when at t = p1.t, based on krum_HMM, there is a transition probability in the matrix
            //when at t = p2.t, based on krum_HMM, there is another transition probability in the matrix
            for (int i = 0; i < _trj.Count - 1; i++)
            {
               
                Point p1 = _trj[i];
                Point p2 = _trj[i + 1];
                
                List<Edge> cands1 = null;
                List<Edge> cands2 = null;

                if (id_candidate.ContainsKey(i))
                {
                    cands1 = id_candidate[i];
                }
                if(id_candidate.ContainsKey(i+1))
                {
                    cands2 = id_candidate[i + 1];
                }

                //searchEdgeWithMaximumRadiusAngle(G._rtree_edge, p1, cands1, search_radius, step, leastNum, maximum_radius, angle_threshod);
                //searchEdgeWithMaximumRadiusAngle(G._rtree_edge, p2, cands2, search_radius, step, leastNum, maximum_radius, angle_threshod);

                if (cands1 == null || cands2 == null)
                    continue;

                bool transition_prob_flag = false;
                foreach (Edge e1 in cands1)
                {
                    foreach (Edge e2 in cands2)
                    {
                        Stopwatch st2 = new Stopwatch();
                        st2.Start();//开始计时

                        double transitionProb = TransitionProbability(p1, e1, p2, e2, G, 0.894,e1_e2_edges);

                        st2.Stop();//终止计时
                        tran_time += st2.ElapsedMilliseconds;

                        if (transitionProb == 0)
                            continue;
                        else
                            transition_prob_flag = true; 

                        #region containner for to from transition
                        if (transition_to_from.ContainsKey(i))
                        {
                            Dictionary<Edge, Dictionary<Edge, double>> e_e_prob = transition_to_from[i];

                            if (e_e_prob.ContainsKey(e2))
                            {
                                Dictionary<Edge, double> e_prob = e_e_prob[e2];
                                if (e_prob.ContainsKey(e1))
                                {
                                    e_prob[e1] = transitionProb;
                                }
                                else
                                {
                                    e_prob.Add(e1, transitionProb);
                                }
                            }
                            else
                            {
                                Dictionary<Edge, double> e_prob = new Dictionary<Edge, double>();
                                e_prob.Add(e1, transitionProb);

                                e_e_prob.Add(e2, e_prob);
                            }
                        }
                        else
                        {
                            Dictionary<Edge, Dictionary<Edge, double>> e_e_prob = new Dictionary<Edge, Dictionary<Edge, double>>();
                            Dictionary<Edge, double> e_prob = new Dictionary<Edge, double>();

                            e_prob.Add(e1, transitionProb);
                            e_e_prob.Add(e2, e_prob);
                            transition_to_from.Add(i, e_e_prob);
                        }

                        #endregion
                    }
                }

                if (transition_prob_flag == false)
                    break;
            }

            #endregion

            
            Stopwatch st3 = new Stopwatch();
            st3.Start();//开始计时

            HMM hmm = new HMM(transition_to_from, emission, initial);

            double prob;
            Edge[] sequence = hmm.Viertbi(false, out prob);
            
            st3.Stop();//终止计时
            viterbi_time = st3.ElapsedMilliseconds;

            if (sequence == null)
                return;
            for (int i = 0; i < sequence.GetLength(0) - 1; i++)
            {
                if (sequence[i].Equals(sequence[i+1]))
                {
                    continue;
                }
                complete_route.Add(sequence[i]);
                if (!sequence[i].isConnectedTo(sequence[i + 1]))
                {
                    if (e1_e2_edges.ContainsKey(sequence[i]))
                    {
                        if (e1_e2_edges[sequence[i]].ContainsKey(sequence[i + 1]))
                        {
                            if (e1_e2_edges[sequence[i]][sequence[i + 1]].Count > 0)
                            {
                                complete_route.AddRange(e1_e2_edges[sequence[i]][sequence[i + 1]]);
                            }
                        }
                    }
                }
            }
            complete_route.Add(sequence[sequence.GetLength(0) - 1]);

            #region convert sequence to road segment with time
            //complete_road_t = new List<Road_Time>();
            //if (sequence != null)
            //{
            //    List<Road_Time> final_road_t = new List<Road_Time>();

            //    List<Road_Time> road_t = new List<Road_Time>();
            //    for (int i = 0; i < sequence.GetLength(0); i++)
            //    {
            //        Road_Time rt = new Road_Time();
            //        rt.road = sequence[i];
            //        rt.T = _trj[i].T;
            //        road_t.Add(rt);
            //    }

            //    GetCompletePath(road_t, ref complete_road_t);

            //    if (complete_road_t.Count > 0)
            //    {
            //        int the_last_road_id = complete_road_t[0].road.ID;
            //        Road_Time rt1 = new Road_Time();
            //        rt1.road = complete_road_t[0].road;
            //        rt1.T = complete_road_t[0].T;
            //        final_road_t.Add(rt1);

            //        for (int i = 1; i < complete_road_t.Count; i++)
            //        {
            //            if (complete_road_t[i].road.ID != the_last_road_id)
            //            {
            //                Road_Time rt = new Road_Time();
            //                rt.road = complete_road_t[i].road;
            //                rt.T = complete_road_t[i].T;

            //                the_last_road_id = complete_road_t[i].road.ID;
            //                final_road_t.Add(rt);
            //            }
            //        }
            //    }

            //    complete_road_t.Clear();

            //    complete_road_t.AddRange(final_road_t);

            //} 
            #endregion

        }

        private void searchEdgeWithinRadiusOneStep(rtree rt, Point p, HashSet<Edge> cands, double radius)
        {
            double xmin, ymin, xmax, ymax;
            ymin = p.Y - radius;
            ymax = p.Y + radius;
            xmin = p.X - radius;
            xmax = p.X + radius;

            MBR rect = new MBR(xmin, ymin, xmax, ymax);

            rt.search(rect, cands);
        }

        private void searchEdgeWithMaximumRadiusAngle(rtree rt, Point p, HashSet<Edge> cands, double radius, double step, int leastNum, double max_radius,double angle)
        {
            double xmin, ymin, xmax, ymax;
            ymin = p.Y - radius;
            ymax = p.Y + radius;
            xmin = p.X - radius;
            xmax = p.X + radius;

            double current_radius = radius;

            while (cands.Count < leastNum && current_radius <= max_radius)
            {
                MBR rect = new MBR(xmin, ymin, xmax, ymax);

                rt.search(rect, cands,p,angle);

                ymin = ymin - step;
                ymax = ymax + step;
                xmin = xmin - step;
                xmax = xmax + step;

                current_radius += step;
            }
        }

        private double MeasurementProbability(Point p, Edge e, double miu_o,double sigma_o,double miu_b,double sigma_b)
        {
            //observation probability
            double p_lng, p_lat;
            p_lng = p.X;
            p_lat = p.Y;

            Point p_prj = e.projectFrom(p);
            double p_prj_lng, p_prj_lat;
            p_prj_lng = p_prj.X;
            p_prj_lat = p_prj.Y;

            double great_circle_dis = p.DistanceFrom(p_prj);

            double measurementProb1, measurementProb2 = 0;
            double var1 = 1.0 / (Math.Sqrt(2 * Math.PI) * sigma_o);
            double var2 = (great_circle_dis-miu_o) / sigma_o;

            //direction analysis
            //double theta = Geometry.getAngle(e);
            double derta = Geometry.getAnglediff(p.Direction,e);

            double var11 = 1.0 / (Math.Sqrt(2 * Math.PI) * sigma_b);
            double var22 = (derta-miu_b) / sigma_b;

            measurementProb1 = var1 * Math.Exp(-0.5 * Math.Pow(var2, 2));
            measurementProb2 = var11 * Math.Exp(-0.5 * Math.Pow(var22, 2));
            //Console.WriteLine("{0},{1}",great_circle_dis,measurementProb);
            return measurementProb1 * measurementProb2*100;
        }

        private double TransitionProbability(Point p1, Edge e1, Point p2, Edge e2, Graph g, double beta, Dictionary<Edge, Dictionary<Edge, List<Edge>>> e1_e2_edges)
        {
            //transition probability
            List<Edge> shortest_path = new List<Edge>();

            Stopwatch st = new Stopwatch();
            st.Start();//开始计时
            Edgespp routes = RouteDistance(p1, e1, p2, e2, g);
            st.Stop();//终止计时
            short_time += st.ElapsedMilliseconds;

            if (routes == null)
                return 0;

            if (e1_e2_edges.ContainsKey(e1))
            {
                if (e1_e2_edges[e1].ContainsKey(e2))
                {
                    e1_e2_edges[e1][e2] = routes.Routes;
                }
                else
                {
                    e1_e2_edges[e1].Add(e2, routes.Routes);
                }
            }
            else
            {
                List<Edge> rs = routes.Routes;
                Dictionary<Edge, List<Edge>> e2_edges = new Dictionary<Edge, List<Edge>>();
                e2_edges.Add(e2, rs);
                e1_e2_edges.Add(e1, e2_edges);

            }

            
            double great_circle_dis = p1.DistanceFrom(p2);
            double transitionProb = great_circle_dis / routes.Dis;

            //temporal analysis
            double avgspeed = routes.Dis / (p2.T - p1.T).TotalSeconds;
            double var1=0, var2=0, var3 = 0;
            
            foreach(Edge r in routes.Routes){
                var1 +=r.Maxspeed * avgspeed;
                var2 += r.Maxspeed * r.Maxspeed;
                var3 += avgspeed * avgspeed;
            }

            double Ft = var1 / (Math.Sqrt(var2) * Math.Sqrt(var3));

            //transitionProb = (1 / beta) * Math.Exp(-1 * deltaDis / beta);
            return transitionProb*Ft;
        }

        private double DirectionCalculate(Edge e,Point p,double sigma,double miu)
        {
            double angle = Math.Atan((e.To.Y - e.From.Y) / (e.To.X - e.From.X));

            double angle_diff = Math.Abs(angle - p.Direction);

            double var1 = 1.0 / (Math.Sqrt(2 * Math.PI) * sigma);
            double var2 = (angle_diff - miu) / sigma;

            double measurementProb = var1 * Math.Exp(-0.5 * Math.Pow(var2, 2));

            return angle_diff;
        }

        /// <summary>
        /// calculate route distance by search the shortest route between the from vertex of the two edges
        /// this method may be not accurate.
        /// <returns></returns>
        private Edgespp RouteDistance(Point p1, Edge e1, Point p2, Edge e2, Graph g)
        {
            double routeDis = 0;

            Point p1_prj = e1.projectFrom(p1);
            Vertex e1_to = e1.To;
            double dis1 = p1_prj.DistanceFrom(e1_to.toPoint());

            Point p2_prj = e2.projectFrom(p2);
            Vertex e2_from = e2.From;
            double dis2 = p2_prj.DistanceFrom(e2_from.toPoint());

            if (e1 == e2)
            {
                return new Edgespp(p1_prj.DistanceFrom(p2_prj),new List<Edge>(){e1});
            }
            else if (e1_to.ID == e2_from.ID)
            {
                routeDis = dis1 + dis2;
                return new Edgespp(routeDis, new List<Edge>() { e1,e2});
            }
            else
            {
                Dijkstra dij = new Dijkstra(g);

                LinkedList<Vertex> shortestPath_vertices = dij.ShortestPath_ScoreValue(e1_to, e2_from, _distance_limit);
                
                //Route rs = getminRoute(F, e1.ID, e2.ID, p1, p2);

                if (shortestPath_vertices == null)
                    return null;

                List<Edge> shortestPath_edges = dij.MakeShortestPath_EdgeList();

                //List<Edge> shortestPath_edges = rs.getPath_EdgeList(m_graph);

                for (int i = 0; i < shortestPath_edges.Count; i++)
                {
                    routeDis += shortestPath_edges[i].Length;
                }

                if (shortestPath_edges.Contains(e1))
                {
                    routeDis -= dis1;
                    shortestPath_edges.Remove(e1);
                }
                else
                {
                    routeDis += dis1;
                }

                if (shortestPath_edges.Contains(e2))
                {
                    routeDis -= dis2;
                    shortestPath_edges.Remove(e2);
                }
                else
                {
                    routeDis += dis2;
                }

                return new Edgespp(routeDis, shortestPath_edges);
            }

        }

        private void GetCompletePath(List<Road_Time> cands, ref List<Road_Time> result)
        {
            result.Add(cands[0]);
            Road_Time rt = new Road_Time();

            bool flag = true;
            for (int i = 0; i < cands.Count - 1; i++)
            {
                if (flag == true)
                {
                    rt = cands[i];
                }

                Road_Time rt_next = cands[i + 1];

                Edge e = cands[i].road;
                Edge next = cands[i + 1].road;

                int eid = e.ID;
                int nextid = next.ID;


                if (eid == nextid)
                {
                    flag = false;
                    continue;
                }

                flag = true;

                if (e.isConnectedTo(next) == true)
                {
                    result.Add(rt_next);
                    continue;
                }
                else
                {
                    //all possible roads in the iterative search procedure.
                    //int: the roadID
                    //int: the last roadID adjacent to the roadID
                    Dictionary<Edge, Edge> possibles = new Dictionary<Edge, Edge>();

                    //the roads are searched out by adjacency in the last round
                    Dictionary<Edge, Edge> lastNewEdges = new Dictionary<Edge, Edge>();

                    possibles.Add(e, null);
                    lastNewEdges.Add(e, null);

                    Dictionary<Edge, Edge> newEdges = new Dictionary<Edge, Edge>();
                    int times = 0; //record the loop times.
                    while (true)
                    {
                        times++;

                        newEdges.Clear();
                        foreach (Edge key in lastNewEdges.Keys)
                        {
                            SearchEdgesForEdge1(key, possibles, ref newEdges);
                        }

                        if (newEdges.ContainsKey(next))
                        {
                            Edge theE = null;
                            newEdges.TryGetValue(next, out theE);
                            possibles.Add(next, theE);
                            break;
                        }

                        lastNewEdges.Clear();

                        foreach (var item in newEdges)
                        {
                            lastNewEdges.Add(item.Key, item.Value);
                            possibles.Add(item.Key, item.Value);
                        }
                    }

                    //if there are n roads between e and next, then divide the timespan (between e and next) into n intervals averagely 
                    TimeSpan ts = rt_next.T - rt.T;
                    int seconds = ts.Seconds;
                    TimeSpan time_interval = TimeSpan.FromSeconds(seconds / times);

                    List<Road_Time> newConnectedEdges = new List<Road_Time>();
                    //Edge edge = nextid;
                    //DateTime lastT = next.T;

                    Edge current_edge = next;
                    DateTime lastT = rt_next.T;
                    while (true)
                    {
                        Edge next_edge;
                        possibles.TryGetValue(current_edge, out next_edge);
                        if (next_edge == e)
                        {
                            break;
                        }
                        else
                        {
                            DateTime t = lastT.Subtract(time_interval);
                            Road_Time theRT = new Road_Time();
                            theRT.road = next_edge;
                            theRT.T = t;
                            theRT.strength = 0;
                            newConnectedEdges.Add(theRT);
                            current_edge = next_edge;
                            lastT = t;
                        }
                    }

                    newConnectedEdges.Reverse();

                    //////////////////////////////////////////////////////////////////////////////////////////////////////////////
                    ////////connected mapped edges of two adjacent gps points can not exceed the max distance the vehicle traveled 
                    double total_distance = 0;
                    for (int j = 0; j < newConnectedEdges.Count; j++)
                    {
                        Road_Time theRT = newConnectedEdges[j];
                        Edge theRoad = theRT.road;
                        total_distance += theRoad.Length;
                    }

                    if ((rt_next.T - rt.T).TotalSeconds * 80 < total_distance)
                    {
                        if (i == 0) //the first gps point may be mapped wrong, need the further check
                        { }
                        else
                        {
                            cands.RemoveAt(i + 1);
                            i = i - 1;
                            continue;
                        }
                    }
                    else
                    {
                        result.AddRange(newConnectedEdges);
                        result.Add(rt_next);
                    }
                }
            }
        }

        private void SearchEdgesForEdge1(Edge e, Dictionary<Edge, Edge> all, ref Dictionary<Edge, Edge> edgeIDs)
        {
            //Edge e = null;
            Vertex v1 = null, v2 = null;

            //e = g.getRoadEdge(edgeID);
            v1 = e.From;
            v2 = e.To;

            if (e == null || v1 == null || v2 == null)
            {
                return;
            }

            List<Edge> adjEdges1 = v1.incidentEdges();
            List<Edge> adjEdges2 = v2.incidentEdges();
            for (int i = 0; i < adjEdges1.Count; i++)
            {
                Edge theEdge = adjEdges1[i];
                if (!(all.ContainsKey(theEdge) || edgeIDs.ContainsKey(theEdge)))
                {
                    edgeIDs.Add(theEdge, e);
                }
            }

            for (int i = 0; i < adjEdges2.Count; i++)
            {
                Edge theEdge = adjEdges2[i];
                if (!(all.ContainsKey(theEdge) || edgeIDs.ContainsKey(theEdge)))
                {
                    edgeIDs.Add(theEdge, e);
                }
            }
        }

        private static double maxDistance(DateTime t1, DateTime t2)
        {
            TimeSpan ts = t2 - t1;
            return ts.TotalSeconds * 60;
        }

        private Route getminRoute(PForest F, int sid, int eid, Point p1, Point p2)
        {
            Dictionary<Route, double> path_dis = new Dictionary<Route, double>();
            double min_len = 99999999;
            Route min_path = null;
            //List<Route> routes = new List<Route>();
            //int f = 0;
            //double v_limit = 0;
            //double time_interval = 0;
            //double road_vlimit = 0;
            //TimeSpan ts = new TimeSpan();
            if (sid == eid)
            {
                //Route p = new Route();
                //p.Add(sid);
                min_path = new Route();
                min_path.Add(sid);
            }
            else
            {
                Tree<Node> tr = F.FindTree(sid);
                TreeNode<Node> curNode = null;
                if (tr != null && tr.Index.ContainsKey(eid))
                {
                    foreach (TreeNode<Node> n in tr.Index[eid])
                    {
                        //road_vlimit = 0;
                        Route pp = new Route();
                        curNode = n;

                        while (curNode != null)
                        {
                            pp.Add(curNode.Data.Sid);
                            curNode = curNode.Parent;
                        }

                        pp.Reverse();

                        #region clean paths
                        //ts = p2.T.Subtract(p1.T).Duration();
                        //time_interval = Convert.ToDouble(ts.TotalSeconds);
                        //v_limit = pp.getLength(g, p1, p2) / time_interval;
                        //foreach (int id in pp)
                        //{
                        //    if (g.getEdgeAttribute(id).ATTRIBUTE[0] == -1)
                        //        road_vlimit += 13;
                        //    else
                        //        road_vlimit += g.getEdgeAttribute(id).ATTRIBUTE[5];
                        //}
                        #endregion

                            //routes.Add(pp);
                        path_dis.Add(pp, pp.getLength(m_graph, p1, p2));
                    }
                }
                else
                {
                    return null;
                }
            }
            var dicSort = from objDic in path_dis orderby objDic.Value select objDic;
            foreach (KeyValuePair<Route, double> k_v in dicSort)
            {
                if (k_v.Value < min_len)
                {
                    min_len = k_v.Value;
                    min_path = k_v.Key;
                }
            }
            return min_path;
        }
    }

    public struct Road_Time
    {
        public int tra_pt_id;
        public Edge road;
        public DateTime T;
        public int strength;
    }
}

