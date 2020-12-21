using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.IO;

namespace HMM_Mapmatch
{
    class HMM
    {
        private int symbols;


        Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>> transitions_to_from;
        Dictionary<Edge, Dictionary<int, double>> emissions;
        Dictionary<Edge, double> initials;

        public HMM(Dictionary<int, Dictionary<Edge, Dictionary<Edge, double>>> transitions,
            Dictionary<Edge, Dictionary<int, double>> emissions, Dictionary<Edge, double> initials)
        {
            symbols = transitions.Count + 1;

            #region Transitions Matrix A
            this.transitions_to_from = transitions;
            #endregion

            #region Initial Probabilities
            this.initials = initials;
            #endregion

            #region Emission Matrix
            this.emissions = emissions;
            #endregion
        }

        public Edge[] Viertbi(bool logarithm, out double probability)
        {
            int T = symbols;

            Edge maxEdge;
            double maxWeight;
            double weight;

            Dictionary<Edge, Dictionary<int, Edge>> s = new Dictionary<Edge, Dictionary<int, Edge>>();
            Dictionary<Edge, Dictionary<int, double>> a = new Dictionary<Edge, Dictionary<int, double>>();//存储第一个GPS点与其候选边之间的观测概率！！！


            #region base
            foreach (Edge e in initials.Keys)
            {
                if (emissions.ContainsKey(e))
                {
                    double ini_prob = initials[e];

                    Dictionary<int, double> obs_prob = emissions[e];

                    if (!obs_prob.ContainsKey(0))
                    {
                        continue;
                    }

                    double measure_prob = obs_prob[0];

                    double prob = ini_prob * measure_prob;   //观测概率 X 转移概率

                    //attention!!!
                    prob = Math.Log10(prob);
                    //attention!!!

                    if (a.ContainsKey(e))
                    {
                        Dictionary<int, double> step_prob = a[e];
                        if (step_prob.ContainsKey(0))
                        {
                            step_prob[0] = prob;
                        }
                        else
                        {
                            step_prob.Add(0, prob);
                        }
                    }
                    else
                    {
                        Dictionary<int, double> step_prob = new Dictionary<int, double>();
                        step_prob.Add(0, prob);
                        a.Add(e, step_prob);
                    }
                }
            }
            #endregion

            #region induction
            for (int t = 1; t < T; t++)
            {
                if (!transitions_to_from.ContainsKey(t - 1))
                {
                    continue;
                }

                Dictionary<Edge, Dictionary<Edge, double>> e_e_prob = transitions_to_from[t - 1];

                maxEdge = null;
                maxWeight = double.MinValue;

                #region detect which step there is no probability to reach.
                bool flag = false;
                #endregion

                foreach (Edge e_to in e_e_prob.Keys)
                {
                    if (!emissions.ContainsKey(e_to)) // the measurement probability of edge_to is zero for all observations
                    {
                        continue;
                    }
                    else if (!emissions[e_to].ContainsKey(t)) //the measurement probability of edge_to is zero for t observation
                    {
                        continue;
                    }

                    double measure_prob = emissions[e_to][t]; //the measurement probability of edge_to for t observation

                    measure_prob = Math.Log10(measure_prob);

                    Dictionary<Edge, double> from_edge_prob = e_e_prob[e_to];

                    maxEdge = null;
                    maxWeight = double.MinValue;
                    foreach (Edge e_from in from_edge_prob.Keys)
                    {
                        if (!a.ContainsKey(e_from)) // the probability at e_from is zero based on previous calculation
                        {
                            continue;
                        }
                        double tran_prob = from_edge_prob[e_from];

                        tran_prob = Math.Log10(tran_prob);

                        Dictionary<int, double> step_prev_prob = a[e_from];

                        if (!step_prev_prob.ContainsKey(t - 1)) // the transition probability from e_from to e_to is zero at the t - 1 step
                        {
                            continue;
                        }
                        double prev_prob = step_prev_prob[t - 1];

                        weight = prev_prob + tran_prob;

                        if (weight > maxWeight)
                        {
                            maxEdge = e_from;
                            maxWeight = weight;
                        }
                    }

                    double current_prob = maxWeight + measure_prob;

                    if (a.ContainsKey(e_to))
                    {
                        Dictionary<int, double> step_prob = a[e_to];
                        if (step_prob.ContainsKey(t))
                        {
                            step_prob[t] = current_prob;
                        }
                        else
                        {
                            step_prob.Add(t, current_prob);
                        }
                    }
                    else
                    {
                        Dictionary<int, double> step_prob = new Dictionary<int, double>();
                        step_prob.Add(t, current_prob);
                        a.Add(e_to, step_prob);
                    }

                    if (s.ContainsKey(e_to))
                    {
                        Dictionary<int, Edge> step_edge = s[e_to];

                        if (step_edge.ContainsKey(t))
                        {
                            step_edge[t] = maxEdge;
                        }
                        else
                        {
                            step_edge.Add(t, maxEdge);
                        }
                    }
                    else
                    {
                        Dictionary<int, Edge> step_edge = new Dictionary<int, Edge>();
                        step_edge.Add(t, maxEdge);
                        s.Add(e_to, step_edge);
                    }

                    if (maxEdge != null)
                    {
                        flag = true;
                    }
                }
            }
            #endregion

            // Find minimum value for time T-1
            maxEdge = null;
            maxWeight = double.MinValue;

            //attention !!! 2016-10-25

            int t_find = T - 1;
            bool flag_find = false;

            for (t_find = T - 1; t_find > 0; t_find--)
            {
                foreach (Edge e in a.Keys)
                {
                    Dictionary<int, double> step_prob = a[e];

                    if (!step_prob.ContainsKey(t_find))
                    {
                        continue;
                    }
                    else
                    {
                        double prob = step_prob[t_find];
                        if (prob > maxWeight)
                        {
                            maxEdge = e;
                            maxWeight = prob;
                            flag_find = true;
                        }
                    }
                }

                if (flag_find == true)
                {
                    break;
                }
            }

            if (flag_find == false)
            {
                probability = 0;
                return null;
            }

            Edge[] path = new Edge[t_find + 1];
            path[t_find] = maxEdge;

            for (int t = t_find - 1; t >= 0; t--)
            {
                if (s.ContainsKey(path[t + 1]))
                {
                    Dictionary<int, Edge> step_edge = s[path[t + 1]];
                    if (!step_edge.ContainsKey(t + 1))
                    {
                    }
                    else
                    {
                        path[t] = step_edge[t + 1];
                    }
                }
            }

            probability = maxWeight;
            return path;

        }

        
    }
}
