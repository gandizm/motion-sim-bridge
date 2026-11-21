"""Build an independent bench fixture and exercise translation, yaw and opening."""
from pathlib import Path
import json
import tempfile
import xml.etree.ElementTree as ET
from replay import run

def main():
    with tempfile.TemporaryDirectory() as tmp:
        canvas=Path(tmp); root=ET.parse(Path(__file__).parent/'assets'/'vehicle.xml')
        world=root.find('worldbody')
        ET.SubElement(world,'geom',name='test_post',type='cylinder',pos='.4 0 .04',size='.018 .04',contype='1',conaffinity='2')
        root.write(canvas/'scene.xml')
        tests={'pass':[(0,0,.32,0,0,0),(1,.42,.32,0,0,0)],
               'contact':[(0,0,0,0,0,0),(1,.23,0,0,0,0)],
               'turn':[(0,.4,-.2,0,0,0),(1,.4,-.2,90,0,0)],
               'open':[(0,.20,-.10,0,0,0),(1,.20,-.10,0,55,-55)]}
        keys=('t_s','x_m','y_m','yaw_deg','left_deg','right_deg')
        for name,segments in tests.items():
            trace=canvas/'trace.jsonl';trace.write_text('\n'.join(json.dumps(dict(zip(keys,p,strict=True))) for p in segments))
            result=run(canvas/'scene.xml',trace)
            print(name,result['samples'],[(c['part'],round(c['t_first_s'],3)) for c in result['contacts']])
            assert bool(result['contacts'])==(name!='pass')
            if name=='open':
                assert any(c['part']=='jaw_left' and c['t_first_s']>0 for c in result['contacts'])
if __name__=='__main__':main()

