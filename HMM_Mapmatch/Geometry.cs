using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HMM_Mapmatch
{
    public interface IVertex
    {
        int ID { get; set; }
        double X { get; set; }
        double Y { get; set; }
        double G { get; set; }
    }

    public class Geometry
    {
        public static DateTime timestamptodt(long timestamp)
        {
            System.DateTime startTime = TimeZone.CurrentTimeZone.ToLocalTime(new System.DateTime(1970, 1, 1)); // 当地时区
            DateTime dt = startTime.AddSeconds(timestamp);
            return dt;
            //System.Console.WriteLine(dt.ToString("yyyy/MM/dd HH:mm:ss"));
        }

        public static double getAngle(Edge e)
        {
            double angle = 0;
            double dis_cos = Math.Cos(Math.PI);//夹角阈值
            //相邻vertex确定的向量
            double vectorA_x = e.To.X - e.From.X;
            double vectorA_y = e.To.Y - e.From.Y;
            //轨迹段首尾vertex确定的向量
            double vectorB_x = 0;
            double vectorB_y = 1;

            double mA = Math.Sqrt(Math.Pow(vectorA_x, 2) + Math.Pow(vectorA_y, 2));
            double mB = Math.Sqrt(Math.Pow(vectorB_x, 2) + Math.Pow(vectorB_y, 2));

            double AB = vectorA_x * vectorB_x + vectorA_y * vectorB_y;

            double anglecos = AB / (mA * mB);

            if(e.From.X<=e.To.X)
                angle = Math.Acos(anglecos) * 180 / Math.PI;
            else
                angle = 360 - Math.Acos(anglecos) * 180 / Math.PI;
           
            return angle;
        }

        public static double getAnglediff(double angle, Edge e)
        {
            double angle_diff = 0;

            if (e.Angle_neg == -1)
            {
                //Console.WriteLine(t.getangle());
                if (Math.Abs(e.Angle_pos - angle) < 180)
                    angle_diff = Math.Abs(e.Angle_pos - angle);
                else
                {
                    angle_diff = 360 - Math.Abs(e.Angle_pos - angle);
                }
            }
            else
            {
                double diff1 = 0, diff2 = 0;

                if (Math.Abs(e.Angle_pos - angle) < 180)
                    diff1 = Math.Abs(e.Angle_pos - angle);
                else
                {
                    diff1 = 360 - Math.Abs(e.Angle_pos - angle);
                }

                if (Math.Abs(e.Angle_neg - angle) < 180)
                    diff2 = Math.Abs(e.Angle_neg - angle);
                else
                {
                    diff2 = 360 - Math.Abs(e.Angle_neg - angle);
                }

                angle_diff = Math.Min(diff1, diff2);

            }

            return angle_diff;
        }
    }

    #region Class Point
    public class Point
    {
        protected double x;
        public double X
        {
            get { return x; }
            set { x = value; }
        }

        protected double y;
        public double Y
        {
            get { return y; }
            set { y = value; }
        }

        protected DateTime t;
        public DateTime T
        {
            get { return t; }
            set { t = value; }
        }
        protected double speed;

        public double Speed
        {
            get { return speed;}
            set { speed = value; }
        }
        protected double direction;

        public double Direction
        {
            get { return direction; }
            set { direction = value; }
        }




        public double DistanceFrom2(double x, double y)
        {
            return (this.x - x) * (this.x - x) + (this.y - y) * (this.y - y);
        }

        public double DistanceFrom2(Point p)
        {
            return DistanceFrom2(p.X, p.Y);
        }

        public static double Distance2(double x1, double y1, double x2, double y2)
        {
            return (x1 - x2) * (x1 - x2) + (y1 - y2) * (y1 - y2);
        }

        public static double Distance2(Point p1, Point p2)
        {
            return (p1.X - p2.X) * (p1.X - p2.X) + (p1.Y - p2.Y) * (p1.Y - p2.Y);
        }

        public double DistanceFrom(double x, double y)
        {
            return Math.Sqrt(DistanceFrom2(x, y));
        }

        public double DistanceFrom(Point p)
        {
            return Math.Sqrt(DistanceFrom2(p));
        }

        public static double Distance(double x1, double y1, double x2, double y2)
        {
            return Math.Sqrt(Distance2(x1, y1, x2, y2));
        }

        public static double Distance(Point p1, Point p2)
        {
            return Math.Sqrt(Distance2(p1, p2));
        }


        public Point()
        {
            t = new DateTime();
            x = 0;
            y = 0;
            speed = 0.0;
            direction = 0.0;
        }

        public Point(double x, double y)
        {
            t = new DateTime();
            this.x = x;
            this.y = y;
        }

        public Point(DateTime t, double x, double y,double direction,double speed)
        {
            this.t = t;
            this.x = x;
            this.y = y;
            this.direction = direction;
            this.speed = speed;
        }

        public Point(Point p)
        {
            this.t = p.t;
            this.x = p.x;
            this.y = p.y;
            this.speed = p.speed;
            this.direction = p.direction;
        }
    }
    #endregion

    #region Class Trajectory
    public class Trajectory : List<Point>
    {
        public Trajectory()
        {
        }
    }
    #endregion

    #region Class Vertex
    public class Vertex : IIndexedObject, IVertex
    {
        protected int id;
        public int ID
        {
            get { return id; }
            set { id = value; }
        }

        protected double x;
        public double X
        {
            get { return x; }
            set { x = value; }
        }

        protected double y;
        public double Y
        {
            get { return y; }
            set { y = value; }
        }

        #region for Shortest path calculation
        public Vertex prev;

        public int Index { get; set; }

        public double g;
        public double G
        {
            get { return g; }
            set { g = value; }
        }
        #endregion

        protected List<Edge> _E;
        public List<Edge> _E_in;
        public List<Edge> _E_out;

        public List<Vertex> _V;
        public List<Vertex> _V_from;
        public List<Vertex> _V_to;   

        public Vertex()
        {
            this.id = -1;
            this.x = -1;
            this.y = -1;

            _E = new List<Edge>();
            _E_in = new List<Edge>();
            _E_out = new List<Edge>();
            _V = new List<Vertex>();
            _V_from = new List<Vertex>();
            _V_to = new List<Vertex>();
        }

        public Vertex(int id)
        {
            this.id = id;
            this.x = -1;
            this.y = -1;

            _E = new List<Edge>();
            _E_in = new List<Edge>();
            _E_out = new List<Edge>();
            _V = new List<Vertex>();
            _V_from = new List<Vertex>();
            _V_to = new List<Vertex>();
        }

        public Vertex(int id, Point p)
        {
            this.id = id;
            this.x = p.X;
            this.y = p.Y;

            _E = new List<Edge>();
            _E_in = new List<Edge>();
            _E_out = new List<Edge>();
            _V = new List<Vertex>();
            _V_from = new List<Vertex>();
            _V_to = new List<Vertex>();
        }

        public void registerEdge(Edge e)
        {
            _E.Add(e);

            if (e.From == this)
            {
                _V.Add(e.To);
                _V_to.Add(e.To);

                _E_out.Add(e);
            }
            else
            {
                _V.Add(e.From);
                _V_from.Add(e.From);

                _E_in.Add(e);
            }
        }

        public void registerEdgeWithAttribute(Edge e, int neglane) //with lane direction information
        {
            _E.Add(e);

            if (e.From == this)
            {
                _V.Add(e.To);
                _V_to.Add(e.To);

                _E_out.Add(e);

                if (neglane > 0)
                {
                    _V_from.Add(e.To);
                    _E_in.Add(e);
                }
            }
            else
            {
                _V.Add(e.From);
                _V_from.Add(e.From);

                _E_in.Add(e);

                if (neglane > 0)
                {
                    _V_to.Add(e.From);
                    _E_out.Add(e);
                }
            }
        }

        public void deRegisterEdge(Edge e)
        {
            _E.Remove(e);

            if (e.From == this)
            {
                _V.Remove(e.To);
                _V_to.Remove(e.To);
                _E_out.Remove(e);
            }
            else
            {
                _V.Remove(e.From);
                _V_from.Remove(e.From);
                _E_in.Remove(e);
            }
        }

        /// <summary>
        /// 
        /// </summary>
        /// <param name="e"></param>
        /// <param name="neglane">1 means the edge is a two-way lane, 0 means the edge is a one-way lane</param>
        public void deRegisterEdgeWithAttribute(Edge e, int neglane)
        {
            _E.Remove(e);

            if (e.From == this)
            {
                _V.Remove(e.To);
                _V_to.Remove(e.To);
                _E_out.Remove(e);

                if (neglane > 0)
                {
                    _V_from.Remove(e.To);
                    _E_in.Remove(e);
                }
            }
            else
            {
                _V.Remove(e.From);
                _V_from.Remove(e.From);
                _E_in.Remove(e);

                if (neglane > 0)
                {
                    _V_to.Remove(e.From);
                    _E_out.Remove(e);
                }
            }
        }

        public void calculateInOut()
        {
            for (int i = 0; i < this._E.Count; i++)
            {
                Edge e = this._E[i];

                if (this.id == e.To.ID)
                {
                    this._E_in.Add(e);
                    this._V_from.Add(e.From);
                }

                if (this.id == e.From.ID)
                {
                    this._E_out.Add(e);
                    this._V_to.Add(e.To);
                }
            }
        }

        public List<Vertex> incidentVertices()
        {
            return _V;
        }

        public List<Edge> incidentEdges()
        {
            return _E;
        }

        public List<Edge> incidentInEdges()
        {
            calculateInOut();
            return _E_in;
        }

        public List<Edge> incidentOutEdges()
        {
            calculateInOut();
            return _E_out;
        }

        public List<Edge> getEdges()
        {
            return _E;
        }

        public List<Edge> getInEdges()
        {
            return _E_in;
        }

        public List<Edge> getOutEdges()
        {
            return _E_out;
        }

        public List<Vertex> adjacentToVertices()
        {
            return _V_to;
        }

        public List<Vertex> adjacentFromVertices()
        {
            return _V_from;
        }

        public double DistanceFrom2(Point p)
        {
            return Point.Distance2(this.x, this.y, p.X, p.Y);
        }

        public Point toPoint()
        {
            return new Point(this.x, this.y);
        }

        public bool Equals(Vertex obj)
        {
            if (this.id == obj.id)
            {
                return true;
            }
            else
            {
                return false;
            }
        }
    }
    #endregion 

    #region Class Edge
    public class Edge
    {
        protected int id;
        public int ID
        {
            get { return id; }
            set { id = value; }
        }

        protected Vertex from;
        public Vertex From
        {
            get { return from; }
            set { from = value; }
        }

        protected Vertex to;
        public Vertex To
        {
            get { return to; }
            set { to = value; }
        }

        private double _length;
        public double Length
        {
            get { return _length; }
            set { _length = value; }
        }

        private double _angle_pos = -1;//道路正方向的角度
        public double Angle_pos
        {
            get { return _angle_pos; }
            set { _angle_pos = value; }
        }

        private double _angle_neg = -1;//道路反方向的角度-如果此值为-1说明为单向道
        public double Angle_neg
        {
            get { return _angle_neg; }
            set { _angle_neg = value; }
        }

        private double minspeed = -1;

        public double Minspeed
        {
            get { return minspeed; }
            set { minspeed = value; }
        }
        private double maxspeed = -1;

        public double Maxspeed
        {
            get { return maxspeed; }
            set { maxspeed = value; }
        }

        public MBR _mbr;
        private bool hasMBR;

        public Edge()
        {
            id = -1;
            hasMBR = false;
        }

        public Edge(int id, Vertex from, Vertex to)
        {
            this.id = id;
            this.from = from;
            this.to = to;
            hasMBR = false;
        }

        public Edge(Vertex from, Vertex to)
        {
            this.id = -1;
            this.from = from;
            this.to = to;

            hasMBR = false;
        }

        private double length()
        {
            _length = Point.Distance(from.X, from.Y, to.X, to.Y);
            return _length;
        }

        public MBR getMBR()
        {
            if (hasMBR)
            {
                return _mbr;
            }
            else
            {
                createMBR();

                return _mbr;
            }
        }

        public void createMBR()
        {
            _mbr = new MBR();
            _mbr.unionWith(this.from);
            _mbr.unionWith(this.to);

            hasMBR = true;
        }

        public Point projectFrom(DateTime t, double x, double y)
        {
            double vx = to.X - from.X;
            double vy = to.Y - from.Y;

            double wx = x - from.X;
            double wy = y - from.Y;

            double bx, by;

            double c1 = wx * vx + wy * vy;
            double c2 = vx * vx + vy * vy;

            double para = -1;
            if (c2 != 0)
            {
                para = c1 / c2;
            }

            if (para < 0)
            {
                bx = from.X;
                by = from.Y;
            }
            else if (para > 1)
            {
                bx = to.X;
                by = to.Y;
            }
            else
            {
                double b = c1 / c2;
                bx = from.X + b * vx;
                by = from.Y + b * vy;
            }

            return new Point(t, bx, by,0,0);
        }

        public Point projectFrom(double x, double y)
        {
            DateTime t = new DateTime();
            return projectFrom(t, x, y);
        }

        public Point projectFrom(Point p)
        {
            return projectFrom(p.T, p.X, p.Y);
        }

        public double DistanceFrom2(double x, double y)
        {
            Point p = projectFrom(x, y);

            Vertex p_x_min, p_x_max;
            if (this.From.X >= this.To.X)
            {
                p_x_max = this.From;
                p_x_min = this.To;
            }
            else
            {
                p_x_min = this.From;
                p_x_max = this.To;
            }

            if (p.X < p_x_min.X)
            {
                Point theP = new Point(x, y);
                return theP.DistanceFrom2(p_x_min.toPoint());
            }
            else if (p.X > p_x_max.X)
            {
                Point theP = new Point(x, y);
                return theP.DistanceFrom2(p_x_max.toPoint());
            }
            else
            {
                return p.DistanceFrom2(x, y);
            }
        }

        public double DistanceFrom2(Point p)
        {
            return DistanceFrom2(p.X, p.Y);
        }

        public int pointAtLineLeftOrRight(Point p)
        {
            double d_v1_x = this.from.X - p.X;
            double d_v1_y = this.from.Y - p.Y;

            double d_v2_x = this.to.X - p.X;
            double d_v2_y = this.to.Y - p.Y;

            double result = d_v1_x * d_v2_y - d_v1_y * d_v2_x;

            if (result > 0)
                return 1;
            else if (result < 0)
                return -1;
            else
                return 0;
        }

        public bool isIntersectWith(Edge e)
        {
            int flag1 = this.pointAtLineLeftOrRight(e.From.toPoint());
            int flag2 = this.pointAtLineLeftOrRight(e.To.toPoint());

            if (flag1 * flag2 <= 0)
                return true;
            else
                return false;
        }

        public bool isIntersectWith(Point p1, Point p2)
        {
            int flag1 = this.pointAtLineLeftOrRight(p1);
            int flag2 = this.pointAtLineLeftOrRight(p2);

            if (flag1 * flag2 <= 0)
                return true;
            else
                return false;
        }

        public bool isIntersectWith(MBR mbr)
        {
            Point left_top = new Point(mbr.XMin, mbr.YMax);
            Point left_down = new Point(mbr.XMin, mbr.YMin);
            Point right_top = new Point(mbr.XMax, mbr.YMax);
            Point right_down = new Point(mbr.XMax, mbr.YMin);

            if (this.isIntersectWith(left_down, right_down))
                return true;

            if (this.isIntersectWith(right_down, right_top))
                return true;

            if (this.isIntersectWith(right_top, left_top))
                return true;

            if (this.isIntersectWith(left_top, left_down))
                return true;

            return false;
        }

        public Vertex getConnectedV(Edge e)
        {
            if (this.From == e.From || this.From == e.To)
                return this.from;
            else if (this.To == e.From || this.To == e.To)
                return this.to;
            else
                return null;
        }
        public bool isConnectedTo(Edge e)
        {
            if (this.From == e.From || this.From == e.To || this.To == e.From || this.To == e.To)
                return true;

            return false;
        }
    }
    #endregion

    #region Class MBR
    public class MBR
    {
        protected bool haspoint;

        protected double xmin;
        public double XMin
        {
            get { return xmin; }
            set { xmin = value; }
        }

        protected double ymin;
        public double YMin
        {
            get { return ymin; }
            set { ymin = value; }
        }

        protected double xmax;
        public double XMax
        {
            get { return xmax; }
            set { xmax = value; }
        }

        protected double ymax;
        public double YMax
        {
            get { return ymax; }
            set { ymax = value; }
        }


        public MBR()
        {
            haspoint = false;
        }

        public MBR(double xmin, double ymin, double xmax, double ymax)
        {
            this.xmin = xmin;
            this.xmax = xmax;
            this.ymin = ymin;
            this.ymax = ymax;
            haspoint = true;
        }

        public MBR(Point p1, Point p2)
        {
            this.xmin = Math.Min(p1.X, p2.X);
            this.ymin = Math.Min(p1.Y, p2.Y);
            this.xmax = Math.Max(p1.X, p2.X);
            this.ymax = Math.Max(p1.Y, p2.Y);
            haspoint = true;
        }

        public MBR(MBR mbr1)
        {
            this.xmin = mbr1.XMin;
            this.xmax = mbr1.XMax;
            this.ymin = mbr1.YMin;
            this.ymax = mbr1.YMax;

            haspoint = true;
        }

        public void unionWith(Point p)
        {
            unionWith(p.X, p.Y);
        }

        public void unionWith(MBR mbr)
        {
            if (haspoint && mbr.haspoint)
            {
                unionWith(mbr.minPoint());
                unionWith(mbr.maxPoint());
            }

            if (haspoint == false && mbr.haspoint == true)
            {
                this.become(mbr);
            }
        }

        public void unionWith(double x, double y)
        {
            if (haspoint)
            {
                xmin = Math.Min(xmin, x);
                ymin = Math.Min(ymin, y);
                xmax = Math.Max(xmax, x);
                ymax = Math.Max(ymax, y);
            }
            else
            {
                xmin = x;
                xmax = x;
                ymin = y;
                ymax = y;
                haspoint = true;
            }
        }

        public void unionWith(Vertex v)
        {
            unionWith(v.X, v.Y);
        }

        public void unionWith(List<Point> pts)
        {
            for (int i = 0; i < pts.Count; i++ )
            {
                unionWith(pts[i]);
            }
        }

        public bool contain(Point p)
        {
            return ((p.X >= xmin) && (p.X <= xmax) && (p.Y >= ymin) && (p.Y <= ymax));
        }

        public bool contain(MBR mbr)
        {
            return ((mbr.XMin >= xmin) && (mbr.XMax <= xmax) && (mbr.YMin >= ymin) && (mbr.YMax <= ymax));
        }

        public double area()
        {
            return ((xmax - xmin) * (ymax - ymin));
        }

        public double width()
        {
            return xmax - xmin;
        }

        public double height()
        {
            return ymax - ymin;
        }

        public Point minPoint()
        {
            return new Point(xmin, ymin);
        }

        public Point maxPoint()
        {
            return new Point(xmax, ymax);
        }

        public void reset()
        {
            haspoint = false;
        }

        public void become(MBR rect)
        {
            xmin = rect.XMin;
            xmax = rect.XMax;
            ymin = rect.YMin;
            ymax = rect.YMax;

            haspoint = true;
        }

        public bool isIntersectWith(MBR other)
        {
            if (Math.Max(xmin, other.XMin) <= Math.Min(xmax, other.XMax) && Math.Max(ymin, other.YMin) <= Math.Min(ymax, other.YMax))
                return true;
            else
                return false;
        }

        public bool isIntersectWith(Edge e)
        {
            return e.isIntersectWith(this);
        }
    }
    #endregion

    #region 候选路径Route
    public class Route : List<int>
    {
        public Route()
        {
        }

        public List<Edge> path_edges = null;

        public double getLength(Graph g, Point p1, Point p2)
        {
            path_edges = this.getPath_EdgeList(g);
            Edge e1 = path_edges[0];
            Edge e11 = path_edges[1];
            Edge e2 = path_edges[path_edges.Count - 2];
            Edge e22 = path_edges[path_edges.Count - 1];
            Point p1_prj = e1.projectFrom(p1);
            Point p2_prj = e22.projectFrom(p2);

            double length = RouteDistance(p1_prj, e1.getConnectedV(e11), p2_prj, e2.getConnectedV(e22), g, e1.ID == e2.ID);

            return length;
        }
        public double getLength(Graph g)
        {
            path_edges = this.getPath_EdgeList(g);
            double length = 0;

            for (int i = 0; i < path_edges.Count; i++)
            {
                length += path_edges[i].Length;
            }
            return length;
        }

        public List<Edge> getPath_EdgeList(Graph g)
        {
            List<Edge> results = new List<Edge>();

            foreach (int id in this)
            {
                results.Add(g.getEdge(id));
            }

            return results;
        }

        public LinkedList<Vertex> getPath_VertexList(Graph g)
        {
            LinkedList<Vertex> results = new LinkedList<Vertex>();
            Edge e1 = g.getEdge(this[0]);
            Edge e2 = g.getEdge(this[1]);
            Edge e = null;
            if (e1.To.Equals(e2.From))
            {
                results.AddLast(e1.From);
                results.AddLast(e1.To);
                results.AddLast(e2.To);
            }
            else if (e1.To.Equals(e2.To))
            {
                results.AddLast(e1.From);
                results.AddLast(e1.To);
                results.AddLast(e2.From);
            }
            else if (e1.From.Equals(e2.To))
            {
                results.AddLast(e1.To);
                results.AddLast(e1.From);
                results.AddLast(e2.From);
            }
            else
            {
                results.AddLast(e1.To);
                results.AddLast(e1.From);
                results.AddLast(e2.To);
            }

            for (int i = 2; i < this.Count; i++)
            {
                e = g.getEdge(this[i]);
                if (results.Last.Value.Equals(e.From))
                {
                    results.AddLast(e.To);
                }
                else
                {
                    results.AddLast(e.From);
                }
            }

            return results;
        }

        public double RouteDistance(Point p1_prj, Vertex v1, Point p2_prj, Vertex v2, Graph g, bool flag)
        {
            double routeDis = 0;
            double dis1 = p1_prj.DistanceFrom(v1.toPoint());
            double dis2 = p2_prj.DistanceFrom(v2.toPoint());

            if (flag)
            {
                return p1_prj.DistanceFrom(p2_prj);
            }
            else
            {
                for (int i = 1; i < path_edges.Count - 1; i++)
                {
                    routeDis += path_edges[i].Length;
                }

                routeDis += dis1;
                routeDis += dis2;

                return routeDis;
            }

        }

        public override bool Equals(object obj)
        {
            Route r = (Route)obj;
            if (r.Count != this.Count)
                return false;
            else
            {
                bool flag = true;
                for (int i = 0; i < r.Count; i++)
                {
                    if (r[i] != this[i])
                    {
                        flag = false;
                        break;
                    }
                }
                if (flag)
                    return true;
                else
                    return false;
            }
        }

        public override int GetHashCode()
        {
            return this.Count.GetHashCode();
        }

    }

    #endregion

    public class Edgespp
    {
        private double dis = 0;

        public double Dis
        {
            get { return dis; }
            set { dis = value; }
        }
        private List<Edge> routes = null;

        public List<Edge> Routes
        {
            get { return routes; }
            set { routes = value; }
        }



        public Edgespp(double dis, List<Edge> rs)
        {
            this.dis = dis;
            this.routes = rs;
        }

    }

    public class Node
    {
        public Node() { }

        private int sid;//数据域

        public int Sid
        {
            get { return sid; }
            set { sid = value; }
        }
        private int cnt;//数据域

        public int Cnt
        {
            get { return cnt; }
            set { cnt = value; }
        }

        public override bool Equals(object obj)
        {
            if (obj.GetType().Equals(this.GetType()))
            {
                Node n = (Node)obj;
                if (n.sid == this.sid)
                {
                    return true;
                }
                else
                {
                    return false;
                }
            }
            return false;
        }

        public override int GetHashCode()
        {
            return this.sid.GetHashCode();
        }
    }

    public class PForest
    {
        //private HashSet<Tree<Node>> F = null;
        private Dictionary<int, Tree<Node>> f = null;
        public Dictionary<int, Tree<Node>> F
        {
            get { return f; }
            set { f = value; }
        }

        public PForest()
        {
            f = new Dictionary<int, Tree<Node>>();
        }

        public void addTree(Tree<Node> t)
        {
            if (!f.ContainsKey(t.Head.Data.Sid))
            {
                this.f.Add(t.Head.Data.Sid, t);
            }
            else
            {
                this.f[t.Head.Data.Sid] = t;
            }
        }

        public Tree<Node> FindTree(int sid)
        {
            Tree<Node> res = null;
            if (f != null)
            {
                if (this.f.ContainsKey(sid))
                {
                    res = this.f[sid];
                }
            }
            return res;

        }

        public int GetFreq(int sid, int eid)
        {
            int frequence = 0;

            if (f.ContainsKey(sid))
            {
                if (f[sid].Index.ContainsKey(eid))
                {
                    List<TreeNode<Node>> nodes = f[sid].Index[eid];
                    foreach (TreeNode<Node> n in nodes)
                    {
                        frequence += n.Data.Cnt;
                    }
                }
            }

            return frequence;
        }

        public TreeNode<Node> FindChild(TreeNode<Node> curNode, int sid)
        {
            TreeNode<Node> target = null;
            if (curNode.Child != null && curNode.Child.Count > 0)
            {
                foreach (TreeNode<Node> n in curNode.Child)
                {
                    if (n.Data.Sid == sid)
                    {
                        target = n;
                        break;
                    }
                }
            }

            return target;
        }
    }
}
