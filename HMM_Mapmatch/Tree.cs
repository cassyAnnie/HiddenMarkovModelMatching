using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HMM_Mapmatch
{
    public struct indexnode
    {
        int key;         //键
        int offset;      //位置
        public indexnode(int key, int offset)
        {
            this.key = key;
            this.offset = offset;
        }

        //键属性
        public int Key
        {
            get { return key; }
            set { key = value; }
        }
        //位置属性
        public int Offset
        {
            get { return offset; }
            set { offset = value; }
        }
    }

    public class Tree<T> where T : class,new()
    {
        private TreeNode<T> head;       //头引用
        public TreeNode<T> Head
        {
            get { return head; }
            set { head = value; }
        }

        private Dictionary<int, List<TreeNode<Node>>> index;

        public Dictionary<int, List<TreeNode<Node>>> Index
        {
            get { return index; }
            set { index = value; }
        }

        public void addIndex(TreeNode<Node> n)
        {
            if (index.ContainsKey(n.Data.Sid))
            {
                if (index[n.Data.Sid] != null)
                {
                    index[n.Data.Sid].Add(n);
                }
                else
                {
                    List<TreeNode<Node>> ns = new List<TreeNode<Node>>();
                    ns.Add(n);
                    index[n.Data.Sid] = ns;
                }
            }
            else
            {
                List<TreeNode<Node>> ns = new List<TreeNode<Node>>();
                ns.Add(n);
                index.Add(n.Data.Sid, ns);
            }
        }

        public Tree()
        {
            head = null;
            index = new Dictionary<int, List<TreeNode<Node>>>();
        }

        public Tree(T val)
        {
            TreeNode<T> p = new TreeNode<T>(val);
            head = p;
        }

        public Tree(T val, List<TreeNode<T>> cs)
        {
            TreeNode<T> p = new TreeNode<T>(val, cs);
            head = p;
        }

        //判断是否是空二叉树
        public bool IsEmpty()
        {
            if (head == null)
                return true;
            else
                return false;
        }

        //获取根结点
        public TreeNode<T> Root()
        {
            return head;
        }

        //获取结点的孩子结点
        public List<TreeNode<T>> GetChild(TreeNode<T> p)
        {
            return p.Child;
        }

        //若p非空 删除p的子树
        public List<TreeNode<T>> DeleteL(TreeNode<T> p)
        {
            if ((p == null) || (p.Child == null))
                return null;
            List<TreeNode<T>> tmp = p.Child;
            p.Child = null;
            return tmp;
        }

        //编写算法 在M叉树中查找值为value的结点

        public TreeNode<Node> Search(TreeNode<Node> root, T value)
        {
            TreeNode<Node> temp = root;
            if (temp == null)
                return null;
            if (temp.Data.Equals(value))
                return temp;
            else
            {
                for (int i = 0; i < root.Child.Count && temp == null; i++)
                {
                    temp = Search(root.Child[i], value);
                }
            }
            return temp;
        }

        //判断是否是叶子结点
        public bool IsLeaf(TreeNode<Node> p)
        {
            if ((p != null) && (p.Child == null))
                return true;
            else
                return false;
        }


        //层次遍历
        public void CFS(TreeNode<Node> root)
        {
            if (root == null)
            {
                return;
            }
            TreeNode<Node> tmp = null;
            Queue<TreeNode<Node>> queue = new Queue<TreeNode<Node>>(100);
            Stack<TreeNode<Node>> stack = new Stack<TreeNode<Node>>(100); //将栈初始化大小为100
            queue.Enqueue(root);
            while (queue.Count != 0)
            {
                //结点出队
                tmp = queue.Dequeue();
                //处理当前结点

                for (int i = 0; i < tmp.Child.Count; i++)
                {

                    queue.Enqueue(tmp.Child[i]); //将子节点入队列
                }
                stack.Push(tmp); //将p入栈
            }

            while (stack.Count != 0) //不为空
            {
                tmp = stack.Pop(); //退栈
            }
        }

        //深度优先遍历
        public void DFS(TreeNode<Node> root, string str, ref string strBest, int level)
        {
            if (root == null)
            {
                return;
            }

            string tmp = str + root.Data.Sid.ToString();

            if (root.Child.Count == 0)
            {
                if (strBest == null || tmp.Length > strBest.Length)
                {
                    strBest = tmp;
                }
            }
            for (int i = 0; i < root.Child.Count; i++)
            {
                DFS(root.Child[i], tmp, ref strBest, level + 1);
            }
        }
    }
}
