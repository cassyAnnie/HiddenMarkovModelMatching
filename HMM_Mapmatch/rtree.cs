using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HMM_Mapmatch
{
    public class rtree
    {
        public int minItems;
        public int maxItems;

        public rtree(int min, int max)
        {
            minItems = min;
            maxItems = max;
            elements = new List<Edge>();
            children = new List<rtree>();
            pnts = new List<Edge>();
            m_br = new MBR();
        }

        rtree parent;

        public List<Edge> elements;
        public List<rtree> children;
        public List<Edge> pnts;

        public MBR m_br;

        public rtree insert(Edge e)
        {
            rtree node = chooseLeaf(this, e);

            if (!node.full())
            {
                node.elements.Add(e);
                node.updateMBR();
            }
            else
            {
                handleOverflowElements(node, e);
            }
            return node.getRoot();
        }

        public rtree insert(rtree node)
        {
            if (full())
            {
                return handleOverflowNodes(node);
            }
            else
            {
                this.children.Add(node);
                node.parent = this;
                this.updateMBR();

                return getRoot();
            }
        }

        public void search(MBR mbr, HashSet<Edge> searchPts)
        {
            if (this.isLeaf())
            {
                foreach (Edge p in this.elements)
                {
                    if (p.getMBR().isIntersectWith(mbr))
                    {
                        if (p.isIntersectWith(mbr))
                        {
                            searchPts.Add(p);
                        }
                    }
                }
            }
            else
            {
                foreach (rtree rt in this.children)
                {
                    if (rt.m_br.isIntersectWith(mbr))
                    {
                        rt.search(mbr, searchPts);
                    }
                }
            }
        }

        public void search(MBR mbr, HashSet<Edge> searchPts,Point p,double angle)
        {
            if (this.isLeaf())
            {
                foreach (Edge e in this.elements)
                {
                    if (e.getMBR().isIntersectWith(mbr))
                    {
                        if (e.isIntersectWith(mbr))
                        {
                            if (Geometry.getAnglediff(p.Direction, e) <= angle)
                            {
                                searchPts.Add(e);
                            }
                        }
                    }
                }
            }
            else
            {
                foreach (rtree rt in this.children)
                {
                    if (rt.m_br.isIntersectWith(mbr))
                    {
                        rt.search(mbr, searchPts, p, angle);
                    }
                }
            }
        }

        protected void quadraticSplitElements(List<Edge> oldItems, Edge newItem, List<Edge> set_one, List<Edge> set_two, MBR br_one, MBR br_two)
        {
            oldItems.Add(newItem);

            int m = 0, n = 1;
            double maxdis = 0;
            for (int i = 0; i < oldItems.Count; i++)
            {
                for (int j = i + 1; j < oldItems.Count; j++)
                {
                    double p1_mid_x = (oldItems[i].From.X + oldItems[i].To.X) / 2;
                    double p1_mid_y = (oldItems[i].From.Y + oldItems[i].To.Y) / 2;
                    double p2_mid_x = (oldItems[j].From.X + oldItems[j].To.X) / 2;
                    double p2_mid_y = (oldItems[j].From.Y + oldItems[j].To.Y) / 2;

                    double dis = Math.Pow(p1_mid_x - p2_mid_x, 2) +
                        Math.Pow(p1_mid_y - p2_mid_y, 2);

                    if (dis > maxdis)
                    {
                        maxdis = dis;
                        m = i;
                        n = j;
                    }
                }
            }

            set_one.Add(oldItems[m]);
            set_two.Add(oldItems[n]);

            br_one.unionWith(oldItems[m].getMBR());
            br_two.unionWith(oldItems[n].getMBR());

            oldItems.RemoveAt(m);

            if (m < n)
            {
                oldItems.RemoveAt(n - 1);
            }
            else
            {
                oldItems.RemoveAt(n);
            }

            while (!(oldItems.Count == 0))
            {
                if (minItems - set_one.Count == oldItems.Count)
                {
                    for (int i = 0; i < oldItems.Count; i++)
                    {
                        set_one.Add(oldItems[i]);
                        br_one.unionWith(oldItems[i].getMBR());

                    }
                    break;
                }
                else if (oldItems.Count == minItems - set_two.Count)
                {
                    for (int i = 0; i < oldItems.Count; i++)
                    {
                        set_two.Add(oldItems[i]);
                        br_two.unionWith(oldItems[i].getMBR());
                    }

                    break;
                }
                int k = 0;

                double s = areaEnlarge(br_one, oldItems[k]);
                double t = areaEnlarge(br_two, oldItems[k]);

                if (s < t)
                {
                    set_one.Add(oldItems[k]);
                    br_one.unionWith(oldItems[k].getMBR());
                }
                else
                {
                    set_two.Add(oldItems[k]);
                    br_two.unionWith(oldItems[k].getMBR());
                }

                oldItems.RemoveAt(k);
            }
        }

        protected void quadraticSplitNodes(List<rtree> oldItems, rtree newItem, List<rtree> set_one, List<rtree> set_two, MBR br_one, MBR br_two)
        {
            oldItems.Add(newItem);

            int m = 0, n = 1;
            double maxArea = areaExpand(oldItems[0].m_br, oldItems[1].m_br);

            for (int i = 0; i < oldItems.Count; i++)
                for (int j = i + 1; j < oldItems.Count; j++)
                {
                    double area = areaExpand(oldItems[i].m_br, oldItems[j].m_br);
                    if (area > maxArea)
                    {
                        maxArea = area;
                        m = i;
                        n = j;
                    }
                }

            set_one.Add(oldItems[m]);
            set_two.Add(oldItems[n]);

            MBR set_one_mbr = new MBR(oldItems[m].m_br);
            MBR set_two_mbr = new MBR(oldItems[n].m_br);

            oldItems.RemoveAt(m);

            if (m < n)
                oldItems.RemoveAt(n - 1);
            else
                oldItems.RemoveAt(n);

            while (!(oldItems.Count == 0))
            {
                if (minItems - set_one.Count == oldItems.Count)
                {
                    for (int i = 0; i < oldItems.Count; i++)
                    {
                        set_one.Add(oldItems[i]);
                        set_one_mbr.unionWith(oldItems[i].m_br);
                    }
                    break;
                }
                else if (oldItems.Count == minItems - set_two.Count)
                {
                    for (int i = 0; i < oldItems.Count; i++)
                    {
                        set_two.Add(oldItems[i]);
                        set_two_mbr.unionWith(oldItems[i].m_br);
                    }
                    break;
                }

                int k = 0;

                double s = areaExpand(set_one_mbr, oldItems[k].m_br);
                double t = areaExpand(set_two_mbr, oldItems[k].m_br);

                if (s < t)
                {
                    set_one.Add(oldItems[k]);
                    set_one_mbr.unionWith(oldItems[k].m_br);
                }
                else
                {
                    set_two.Add(oldItems[k]);
                    set_two_mbr.unionWith(oldItems[k].m_br);
                }

                oldItems.RemoveAt(k);
            }

            br_one.become(set_one_mbr);
            br_two.become(set_two_mbr);
        }

        protected rtree buildNode(List<Edge> input, MBR newMbr)
        {
            rtree ret = new rtree(minItems, maxItems);

            foreach (Edge pt in input)
            {
                ret.elements.Add(pt);
            }

            ret.m_br = new MBR(newMbr);

            return ret;
        }

        protected rtree buildNode(List<rtree> input, MBR newMbr)
        {
            rtree ret = new rtree(minItems, maxItems);

            for (int i = 0; i < input.Count; i++)
            {
                ret.children.Add(input[i]);
                ret.children[i].parent = ret;
            }

            if (newMbr != null)
                ret.m_br = new MBR(newMbr);

            return ret;
        }

        protected void setElement(List<Edge> newElements, MBR newMbr)
        {
            this.elements.Clear();
            foreach (Edge pt in newElements)
            {
                this.elements.Add(pt);
            }

            this.children.Clear();

            if (m_br == null)
                m_br = new MBR(newMbr);
            else
                m_br.become(newMbr);
        }

        protected rtree makeNewRoot(rtree child)
        {
            rtree newRoot = new rtree(minItems, maxItems);
            newRoot.children.Add(this);

            newRoot.children.Add(child);

            this.parent = newRoot;
            child.parent = newRoot;
            newRoot.updateMBRLocal();

            return newRoot;
        }

        protected rtree handleOverflowElements(rtree node, Edge e)
        {
            List<Edge> set_one = new List<Edge>();
            List<Edge> set_two = new List<Edge>();

            MBR mbr_one = new MBR();
            MBR mbr_two = new MBR();

            List<Edge> oldItems = new List<Edge>();

            foreach (Edge p in node.elements)
            {
                oldItems.Add(p);
            }

            quadraticSplitElements(oldItems, e, set_one, set_two, mbr_one, mbr_two);

            rtree child_one = buildNode(set_one, mbr_one);

            node.setElement(set_two, mbr_two);

            if (node.isRoot())
            {
                return node.makeNewRoot(child_one);
            }
            else
            {
                return node.parent.insert(child_one);
            }

        }

        protected rtree handleOverflowNodes(rtree node)
        {
            List<rtree> set_one = new List<rtree>();
            List<rtree> set_two = new List<rtree>();

            MBR mbr_one = new MBR();
            MBR mbr_two = new MBR();

            List<rtree> oldItems = new List<rtree>();

            foreach (rtree tr in this.children)
            {
                oldItems.Add(tr);
            }

            this.children.Clear();

            quadraticSplitNodes(oldItems, node, set_one, set_two, mbr_one, mbr_two);

            rtree child_one = buildNode(set_one, mbr_one);

            setChildren(set_two, mbr_two);

            if (this.isRoot())
                return makeNewRoot(child_one);
            else
                return this.parent.insert(child_one);


        }

        protected void setChildren(List<rtree> newChildren, MBR newMbr)
        {
            if (newChildren.Count == 1)
            {
                if (newChildren[0].isLeaf())
                    this.setElement(newChildren[0].elements, newChildren[0].m_br);
                else
                    this.setChildren(newChildren[0].children, newChildren[0].m_br);

                newChildren[0].children.Clear();
                newChildren[0].elements.Clear();

                return;
            }

            this.children.Clear();

            foreach (rtree rt in newChildren)
            {
                rt.parent = this;
                this.children.Add(rt);
            }

            this.elements.Clear();

            if (newMbr != null)
            {
                if (this.m_br == null)
                    this.m_br = new MBR(newMbr);
                else
                    this.m_br.become(newMbr);
            }
        }

        protected rtree getRoot()
        {
            if (!isRoot())
                return parent.getRoot();
            else
                return this;
        }

        protected bool isLeaf()
        {
            if (this.children.Count == 0)
                return true;
            else
                return false;
        }

        protected bool full()
        {
            if (this.isLeaf())
            {
                if (maxItems <= this.elements.Count)
                    return true;
                else
                    return false;
            }
            else
            {
                if (maxItems <= this.children.Count)
                    return true;
                else
                    return false;
            }
        }

        protected double areaEnlarge(MBR br, Edge e)
        {
            double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
            if (e.getMBR().XMax > br.XMax)
                xmax = e.getMBR().XMax;
            else
                xmax = br.XMax;

            if (e.getMBR().XMin < br.XMin)
                xmin = e.getMBR().XMin;
            else
                xmin = br.XMin;

            if (e.getMBR().YMax > br.YMax)
                ymax = e.getMBR().YMax;
            else
                ymax = br.YMax;

            if (e.getMBR().YMin < br.YMin)
                ymin = e.getMBR().YMin;
            else
                ymin = br.YMin;

            return (ymax - ymin) * (xmax - xmin) - br.area();
        }

        protected double areaExpand(MBR br1, MBR br2)
        {
            double xmin = 0, xmax = 0, ymin = 0, ymax = 0;
            if (br1.XMax > br2.XMax)
                xmax = br2.XMax;
            else
                xmax = br2.XMax;

            if (br1.XMin < br2.XMin)
                xmin = br1.XMin;
            else
                xmin = br2.XMin;

            if (br1.YMax > br2.YMax)
                ymax = br1.YMax;
            else
                ymax = br2.YMax;

            if (br1.YMin < br2.YMin)
                ymin = br1.YMin;
            else
                ymin = br2.YMin;

            return (ymax - ymin) * (xmax - xmin) - br1.area() - br2.area();
        }

        protected rtree chooseLeaf(rtree v, Edge e)
        {
            if (v.isLeaf())
            {
                return v;
            }

            double areamin = areaEnlarge(v.children[0].m_br, e);
            int j = 0;

            for (int i = 1; i < v.children.Count; i++)
            {
                double area1 = areaEnlarge(v.children[i].m_br, e);
                if (area1 < areamin)
                {
                    j = i;
                    areamin = area1;
                }
            }

            return chooseLeaf(v.children[j], e);
        }

        protected void updateMBRLocal()
        {
            if (this.isLeaf())
            {
                this.m_br = null;

                foreach (Edge e in elements)
                {
                    if (this.m_br == null)
                        this.m_br = new MBR(e.From.toPoint(), e.To.toPoint());
                    else
                        this.m_br.unionWith(e.getMBR());
                }
            }
            else
            {
                this.m_br = null;
                foreach (rtree rt in children)
                {
                    if (this.m_br == null)
                        this.m_br = new MBR(rt.m_br);
                    else
                        this.m_br.unionWith(rt.m_br);
                }
            }
        }

        protected bool isRoot()
        {
            if (this.parent == null)
                return true;
            else
                return false;
        }

        protected void updateMBR()
        {
            updateMBRLocal();

            if (!this.isRoot())
            {
                this.parent.updateMBR();
            }
        }
    }
}
