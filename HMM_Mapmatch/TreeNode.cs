using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace HMM_Mapmatch
{
    public class TreeNode<T> where T : class,new()
    {
        private T data;               //数据域
        private List<TreeNode<T>> child;   //子节点,最多3个节点
        private TreeNode<T> parent;   //父节点


        public TreeNode(T val, List<TreeNode<T>> p)
        {
            data = val;
            child = p;
        }

        public TreeNode(List<TreeNode<T>> p)
        {
            data = new T();
            child = p;
        }

        public TreeNode(T val)
        {
            data = val;
            child = null;
        }

        public TreeNode()
        {
            data = new T();
            child = null;
        }

        public T Data
        {
            get { return data; }
            set { data = value; }
        }

        public List<TreeNode<T>> Child
        {
            get { return child; }
            set { child = value; }
        }

        public TreeNode<T> Parent
        {
            get { return parent; }
            set { parent = value; }
        }

        public void addChild(TreeNode<T> n)
        {
            if (this.child != null)
            {
                this.child.Add(n);
            }
            else
            {
                this.child = new List<TreeNode<T>>();
                this.child.Add(n);
            }
        }
    }
}
