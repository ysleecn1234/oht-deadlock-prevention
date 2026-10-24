import unittest
from types import SimpleNamespace as NS
from hoisting import Hoisting, NODES

class Rules(unittest.TestCase):
    def vehicle(self):
        return NS(job=NS(lot_id=7,leg_index=2,job_id=77,from_node='a',to_node='z'),state='to_dest')

    def test_mode_and_repeatability(self):
        for mode in range(1,4):
            a,b=Hoisting(mode),Hoisting(4)
            for lot in range(100):
                self.assertEqual(a.decision(lot,2,NODES[mode-1],1),b.decision(lot,2,NODES[mode-1],1))
        self.assertEqual(Hoisting(0).plan(self.vehicle(),['a',NODES[0],'z']),{})

    def test_replan_and_visit_counts(self):
        h=Hoisting(4,1); v=self.vehicle(); path=['a',NODES[0],'x',NODES[0],'z']
        first=h.plan(v,path)
        self.assertEqual(first,h.plan(v,path))
        self.assertEqual([i['visit'] for i in first.values()],[1,2])
        v._hoist_visits[NODES[0]]=1
        self.assertEqual(h.plan(v,['x',NODES[0],'z'])[1]['visit'],2)
        v.state='to_pickup'; self.assertFalse(h.plan(v,path))

    def test_destination_exclusion_and_braking(self):
        v=self.vehicle(); h=Hoisting(4,1)
        v.job.to_node=NODES[0]
        self.assertFalse(h.plan(v,['a',NODES[0]]))
        v.job.to_node='z'; plan=h.plan(v,['a','b',NODES[0],'z'])
        self.assertEqual(h.speed_limit(plan,['a','b',NODES[0],'z'],2,lambda a,b:1000,3000),0)
        self.assertAlmostEqual(h.speed_limit(plan,['a','b',NODES[0],'z'],1,lambda a,b:1000,3000),(6000000)**.5)

    def test_probability_extremes_and_validation(self):
        self.assertFalse(Hoisting(4,0).decision(1,1,NODES[0],1)[0])
        self.assertTrue(Hoisting(4,1).decision(1,1,NODES[0],1)[0])
        for args in [(5,), (4,-1), (4,1,0)]:
            with self.assertRaises(ValueError): Hoisting(*args)

if __name__=='__main__': unittest.main()
