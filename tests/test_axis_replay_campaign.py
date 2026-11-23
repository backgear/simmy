import csv
import io
from pathlib import Path
import sys
import tempfile
from types import SimpleNamespace
import unittest
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'examples'))
from axis_replay_campaign import extract_windows,schedule,run


class ExternalReplayTests(unittest.TestCase):
    def write_csv(self,path,change=None):
        rows=[dict(seq=i+100,physical_torque=(-1)**i*500,enabled='1',fault='none') for i in range(400)]
        if change:change(rows)
        with path.open('w',newline='') as f:
            w=csv.DictWriter(f,fieldnames=list(rows[0]));w.writeheader();w.writerows(rows)
        return rows

    def test_exact_unscaled_samples_and_zero_boundaries(self):
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'monitor.csv';rows=self.write_csv(p);bank=extract_windows(p)
            self.assertEqual(len(bank),2)
            self.assertEqual(bank[0]['commands'],[0]+[r['physical_torque'] for r in rows[:200]]+[0])
            self.assertEqual(bank[0]['command_sha256'],bank[1]['command_sha256'])
            self.assertEqual(bank[0]['split'],bank[1]['split'])

    def test_source_gaps_faults_disabled_and_envelope_rejected(self):
        changes=[lambda r:r[20].update(seq=5000),lambda r:r[20].update(enabled='0'),
                 lambda r:r[20].update(fault='stall'),lambda r:r[20].update(physical_torque=6554),
                 lambda r:[x.update(physical_torque=2000) for x in r[:200]],
                 lambda r:[x.update(physical_torque=0) for x in r[:200]]]
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'monitor.csv'
            for change in changes:
                with self.subTest(change=change):
                    self.write_csv(p,change);bank=extract_windows(p)
                    self.assertEqual(len(bank),1);self.assertEqual(bank[0]['csv_row_offset'],200)

    def test_schedule_covers_bank_before_repeating_and_is_reproducible(self):
        order=schedule(53,1000,23250);self.assertEqual(order,schedule(53,1000,23250))
        self.assertEqual(len(order),1000)
        for a in range(0,954,53):self.assertEqual(sorted(order[a:a+53]),list(range(53)))

    def test_archived_input_reproduces_command_order(self):
        import json
        with tempfile.TemporaryDirectory() as d:
            root=Path(d);source=root/'source';source.mkdir();self.write_csv(source/'monitor.csv')
            def args(src,out):return SimpleNamespace(source_root=src,output=out,resume=False,count=10,seed=23250,execute_powered=False,port=None)
            run(args(source,root/'first'));run(args(root/'first/external-source',root/'second'))
            a=json.loads((root/'first/campaign-plan.json').read_text());b=json.loads((root/'second/campaign-plan.json').read_text())
            self.assertEqual(a['order'],b['order'])
            self.assertEqual([x['commands'] for x in a['bank']],[x['commands'] for x in b['bank']])


if __name__=='__main__':unittest.main()
