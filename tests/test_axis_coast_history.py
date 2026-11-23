import sys
from pathlib import Path
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_coast_history_acquire import plans,indexed_plans


class CoastHistoryPlansTests(unittest.TestCase):
    def test_reproducible_bounded_and_balanced(self):
        cases=plans();self.assertEqual(cases,plans())
        self.assertEqual(len(cases),17)
        self.assertEqual(sum(len(c['moves']) for c in cases),96)
        counts={}
        for c in cases:
            u=c['commands'];moves=c['moves'];counts[c['group']]=counts.get(c['group'],0)+len(moves)
            self.assertEqual(len(u),600);self.assertEqual((u[0],u[-1]),(0,0))
            self.assertLessEqual(max(map(abs,u)),6553)
            for a,b in zip(moves[::2],moves[1::2]):
                self.assertEqual(a['code'],-b['code']);self.assertEqual(a['width'],b['width'])
            for m in moves:
                self.assertLessEqual(abs(m['code'])*m['width']*.02/32767,.00601)
                self.assertEqual(u[m['start']:m['start']+m['width']],[m['code']]*m['width'])
                self.assertLessEqual(m['width'],10)
        self.assertEqual(counts,dict(history=24,low_primary=40,weak_hold=32))

    def test_indexed_holds_sample_multiple_sites(self):
        cases=indexed_plans();self.assertEqual(cases,indexed_plans())
        self.assertEqual(len(cases),9);self.assertEqual(sum(len(c['moves']) for c in cases),54)
        for c in cases:
            self.assertEqual((c['commands'][0],c['commands'][-1]),(0,0))
            self.assertLessEqual(max(map(abs,c['commands'])),3276)
            for j in (0,3):
                index,a,b=c['moves'][j:j+3]
                self.assertEqual(index['group'],'index');self.assertEqual(index['width'],1)
                self.assertEqual(a['code'],-b['code']);self.assertEqual(a['width'],10)
                self.assertEqual((index['site'],a['site']),(a['site'],b['site']))


if __name__=='__main__':unittest.main()
