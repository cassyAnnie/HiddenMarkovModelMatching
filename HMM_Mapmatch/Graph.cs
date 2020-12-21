using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace HMM_Mapmatch
{
    public interface IAttribute
    {
        int ID { get; set; }

        List<double> ATTRIBUTE { get; set; }
    }

    public class Attribute : IAttribute
    {
        protected int _id;
        public int ID
        {
            get { return _id; }
            set { _id = value; }
        }
        protected double _distance;
        public double Distance
        {
            get { return _distance; }
            set { _distance = value; }
        }
        protected List<double> _attribute;
        public List<double> ATTRIBUTE
        {
            get { return _attribute; }
            set
            {
                _attribute = new List<double>();
                _attribute.AddRange(value);
            }
        }

        public static bool operator ==(Attribute attr1, Attribute attr2)
        {
            for (int i = 0; i < attr1.ATTRIBUTE.Count; i++)
            {
                if (attr1.ATTRIBUTE[i] != attr2.ATTRIBUTE[i])
                {
                    return false;
                }
            }
            return true;
        }

        public static bool operator !=(Attribute attr1, Attribute attr2)
        {
            for (int i = 0; i < attr1.ATTRIBUTE.Count; i++)
            {
                if (attr1.ATTRIBUTE[i] != attr2.ATTRIBUTE[i])
                {
                    return true;
                }
            }
            return false;
        }

    }

    public class Graph
    {
        private Dictionary<int, Vertex> _V;

        private Dictionary<int, Edge> _E;

        private Dictionary<int, Attribute> _E_Attr; //attribute for edge

        MBR _ext;

        public rtree _rtree_edge;

        public Graph(string vfile, string efile)
        {
            _ext = new MBR();
            load(vfile, efile);


            int min_item = 10;
            int max_item = 20;
            build_rtree_index(min_item, max_item);
        }

        public Graph(string vfile, string efile, string edge_attribute_file)
        {
            _ext = new MBR();

            load(vfile, efile, edge_attribute_file);

            int min_item = 10;
            int max_item = 20;
            build_rtree_index(min_item, max_item);
        }

        private void build_rtree_index(int minItem, int maxItem)
        {
            if (_rtree_edge == null)
                _rtree_edge = new rtree(minItem, maxItem);

            foreach (int eid in _E.Keys)
            {
                Edge e = _E[eid];
                e.createMBR();
                _rtree_edge =  _rtree_edge.insert(e);
            }
        }

        private void loadVertices(string vfile)
        {
            if (_V == null)
                _V = new Dictionary<int,Vertex>();
            else
                _V.Clear();

            System.IO.FileStream fs = new System.IO.FileStream(vfile, FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);
            string strLine = sr.ReadLine();
            while (strLine != null)
            {
                // 这里处理每一行   
                string[] items = strLine.Split(new char[] { '\t' });
                Vertex v = new Vertex();
                v.ID = int.Parse(items[0]);
                v.X = double.Parse(items[1]);
                v.Y = double.Parse(items[2]);
                //v.type = int.Parse(items[3]);
                //v.entrance = int.Parse(items[4]);

                _ext.unionWith(v.X, v.Y);
                _V.Add(v.ID, v);

                strLine = sr.ReadLine();
            }

            sr.Close();
            fs.Close();
        }

        private void loadEdges(string efile)
        {
            if (_E == null)
                _E = new Dictionary<int,Edge>();
            else
                _E.Clear();

            System.IO.FileStream fs = new System.IO.FileStream(efile, FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);
            string strLine = sr.ReadLine();
            while (strLine != null)
            {
                // 这里处理每一行   
                string[] items = strLine.Split(new char[] { '\t' });
                Edge e = new Edge();
                e.ID = int.Parse(items[0]);
                e.From = _V[int.Parse(items[1])];
                e.To = _V[int.Parse(items[2])];

                _E.Add(e.ID, e);

                Vertex v1, v2;
                _V.TryGetValue(e.From.ID, out v1);
                v1.registerEdge(e);

                _V.TryGetValue(e.To.ID, out v2);
                v2.registerEdge(e);

                strLine = sr.ReadLine();
            }

            sr.Close();
            fs.Close();
        }

        private void loadEdgesWithLaneInformation(string efile, string e_attribute_file)
        {
            loadEdgeAttirbutes(e_attribute_file);

            if (_E == null)
                _E = new Dictionary<int, Edge>();
            else
                _E.Clear();

            System.IO.FileStream fs = new System.IO.FileStream(efile, FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);
            string strLine = sr.ReadLine();
            while (strLine != null)
            {
                // 这里处理每一行   
                string[] items = strLine.Split(new char[] { '\t' });
                Edge e = new Edge();
                e.ID = int.Parse(items[0]);
                e.From = _V[int.Parse(items[1])];
                e.To = _V[int.Parse(items[2])];
                //e.type = int.Parse(items[3]);
                Attribute e_attribute = _E_Attr[e.ID];
                
                e.Minspeed=e_attribute.ATTRIBUTE[2];
                e.Maxspeed=e_attribute.ATTRIBUTE[4];
                _E.Add(e.ID, e);


                e.Angle_pos = Geometry.getAngle(e);
                int neglane = int.Parse(e_attribute.ATTRIBUTE[1].ToString());
                if (neglane > 0)//如果为双向道则再计算反方向的角度
                {
                    if (Geometry.getAngle(e) >= 0 && Geometry.getAngle(e) < 180)
                        e.Angle_neg = Geometry.getAngle(e) + 180;
                    else
                        e.Angle_neg = Geometry.getAngle(e) - 180;
                }
                Vertex v1, v2;
                _V.TryGetValue(e.From.ID, out v1);
                v1.registerEdgeWithAttribute(e, neglane);

                _V.TryGetValue(e.To.ID, out v2);
                v2.registerEdgeWithAttribute(e, neglane);

                strLine = sr.ReadLine();
            }

            sr.Close();
            fs.Close();
        }

        private void loadEdgeAttirbutes(string e_attributefile)
        {
            if (_E_Attr == null)
                _E_Attr = new Dictionary<int, Attribute>();
            else
                _E_Attr.Clear();

            System.IO.FileStream fs = new System.IO.FileStream(e_attributefile, FileMode.Open, FileAccess.Read);
            StreamReader sr = new StreamReader(fs);
            string strLine = sr.ReadLine();
            while (strLine != null)
            {
                // 这里处理每一行   
                string[] items = strLine.Split(new char[] { '\t' });
                Attribute e_attribute = new Attribute();
                int eid = int.Parse(items[0]);
                double distance = double.Parse(items[3]);
                int e_poslane = int.Parse(items[2]);
                int e_neglane = int.Parse(items[3]);
                double pos_minspeed = Math.Min(double.Parse(items[4]) / 3.6, double.Parse(items[6]) / 3.6);
                double neg_minspeed = Math.Min(double.Parse(items[5]) / 3.6, double.Parse(items[7]) / 3.6);
                double pos_maxspeed = Math.Max(double.Parse(items[4]) / 3.6, double.Parse(items[6]) / 3.6);
                double neg_maxspeed = Math.Max(double.Parse(items[5]) / 3.6, double.Parse(items[7]) / 3.6);

                e_attribute.ID = eid;
                List<double> values = new List<double>();
                values.Add(e_poslane);
                values.Add(e_neglane);
                values.Add(pos_minspeed);
                values.Add(neg_minspeed);
                values.Add(pos_maxspeed);
                values.Add(neg_maxspeed);
                e_attribute.ATTRIBUTE = values;

                _E_Attr.Add(eid, e_attribute);

                strLine = sr.ReadLine();
            }

            sr.Close();
            fs.Close();
        }

        public void load(string vfile, string efile)
        {
            loadVertices(vfile);
            loadEdges(efile);
        }

        public void load(string vfile, string efile, string e_attribute_file)
        {
            loadVertices(vfile);
            loadEdgesWithLaneInformation(efile, e_attribute_file);
        }

        public MBR getMBR()
        {
            return _ext;
        }

        public int numVertices()
        {
            return _V.Count();
        }

        public Vertex getVertex(int id)
        {
            return _V[id];
        }

        public Dictionary<int, Vertex> getVertices()
        {
            return _V;
        }

        public int numEdges()
        {
            return _E.Count;
        }

        public Edge getEdge(int id)
        {
            if (_E.ContainsKey(id))
            {
                return _E[id];
            }
            else
            {
                return null;
            }
        }

        public Attribute getEdgeAttribute(int id)
        {
            if (_E_Attr.ContainsKey(id))
            {
                return _E_Attr[id];
            }
            else
            {
                return null;
            }
        }

        public Dictionary<int, Edge> getEdge()
        {
            return _E;
        }

        public Dictionary<int, Attribute> getEdgeAttribute()
        {
            return _E_Attr;
        }

        public void getIncidentEdges(Edge e, HashSet<Edge> cands, int recursion)
        {
            int i = recursion;
            HashSet<Vertex> vts = new HashSet<Vertex>();
            List<Edge> eds = new List<Edge>() ;

            cands.Add(e);

            while (i > 0)
            {
                vts.Clear();
                foreach (Edge edge in cands)
                {
                    vts.Add(edge.From);
                    vts.Add(edge.To);
                }

                foreach (Vertex vertex in vts)
                {
                    eds = vertex.incidentEdges();

                    for (int k = 0; k < eds.Count; k++)
                    {
                        cands.Add(eds[k]);
                    }
                }

                i--;
            }
        }

        public void getIncidentEdge(Edge e, HashSet<Edge> cands, int from_recursion, int to_recursion)
        {
            HashSet<Vertex> vts = new HashSet<Vertex>();
            List<Edge> eds = new List<Edge>();

            cands.Add(e);

            while (to_recursion > 0 || from_recursion > 0)
            {
                vts.Clear();

                foreach (Edge edge in cands)
                {
                    if (from_recursion > 0)
                    {
                        vts.Add(edge.From);
                    }

                    if (to_recursion > 0)
                    {
                        vts.Add(edge.To);
                    }
                }

                foreach (Vertex vertex in vts)
                {
                    if (from_recursion > 0 && to_recursion > 0)
                    {
                        eds = vertex.incidentEdges();
                    }
                    else if (from_recursion > 0)
                    {
                        eds = vertex.incidentInEdges();
                    }
                    else
                    {
                        eds = vertex.incidentOutEdges();
                    }

                    for (int k = 0; k < eds.Count; k++)
                    {
                        cands.Add(eds[k]);
                    }
                }

                to_recursion--;
                from_recursion--;
            }
        }

        public void UpdateEdgeLength()
        {
            foreach (int rid in _E.Keys)
            {
                Edge e = _E[rid];

                Vertex v1 = _V[e.From.ID];
                Vertex v2 = _V[e.To.ID];

                e.Length = Math.Sqrt((v1.X - v2.X) * (v1.X - v2.X) + (v1.Y - v2.Y) * (v1.Y - v2.Y));
            }
        }
    }
}
