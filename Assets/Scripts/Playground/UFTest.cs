using UnityEngine;
using System.Collections.Generic;

namespace Telescopes
{
    public class UFTest : MonoBehaviour
    {
        UnionFind<UFIntNode> uf;

        // Use this for initialization
        void Start()
        {
            int count = 20;
            uf = new UnionFind<UFIntNode>(count);
            for (int i = 0; i < uf.NumNodes; i++)
            {
                uf[i].Number = i;
                uf[i].Index = i;
                uf[i].equivClass = new List<int>();
            }

            for (int i = 0; i < 10; i++)
            {
                int x1 = Mathf.FloorToInt(Random.value * 20f);
                int x2 = Mathf.FloorToInt(Random.value * 20f);

                uf.Union(x1, x2);
            }

            for (int i = 0; i < uf.NumNodes; i++)
            {
                UFIntNode root = uf.Find(i);
                root.equivClass.Add(i);
            }

            HashSet<int> seen = new HashSet<int>();

            for (int i = 0; i < uf.NumNodes; i++)
            {
                UFIntNode root = uf.Find(i);
                if (seen.Contains(root.Index)) continue;
                seen.Add(root.Index);

                string s = "";
                foreach (int x in root.equivClass)
                {
                    s += x + " ";
                }

                Debug.Log("Equiv class [" + s + "], sum = " + root.Number);
            }
        }

        // Update is called once per frame
        void Update()
        {

        }
    }


    class UFIntNode : UnionFindNode
    {
        public int Number;
        public int Index;

        public List<int> equivClass;

        public override void Merge(UnionFindNode child)
        {
            UFIntNode cNode = child as UFIntNode;

            Number += cNode.Number;
        }
    }
}