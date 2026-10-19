import unittest
from fractions import Fraction as F
import json
from pathlib import Path
from point_mass_saved_audit import numerical_integral, reference_integral, per_interval_errors
from scipy.integrate import quad
from point_mass_certificate import (certify, polynomial_bounds, sqrt_bounds,
    flow, interval_difference, tightening, rounding_flow_error, certify_input, rounding_radius)


class CertificateTests(unittest.TestCase):
    def test_bernstein_and_sqrt(self):
        lo,hi=polynomial_bounds([F(1),F(-4),F(4)],F(0),F(1))
        self.assertLessEqual(lo,0); self.assertGreaterEqual(hi,1)
        for n in (F(0),F(2),F(1,10**40)):
            a,b=sqrt_bounds(n); self.assertLessEqual(a*a,n); self.assertGreaterEqual(b*b,n)

    def test_inactive_and_constant_active(self):
        for pos,expected in (([0,0,10],0),([0,0,2],.01)):
            c=certify(pos+[0,0,0],[0,0,9.81],1,.02,obstacle=(0,0,0),radius=2,
                      margins=(.2,0),relative_width=F(1,10**6))
            self.assertLessEqual(c['integral']['lower'],expected)
            self.assertGreaterEqual(c['integral']['upper'],expected)
            self.assertEqual(c['integral_status'],'pass')

    def test_narrow_positive_and_boundary_crossing(self):
        # Straight line near sphere, positive density confined near t=.51.
        for y in (1.9999,1.9,2.):
            x=[-5.1,y,10,10,0,0]; obs=(0,0,10)
            c=certify(x,[0,0,9.81],1,1e-12,obstacle=obs,radius=2)
            f=lambda t:max(0.,2-((10*t-5.1)**2+y*y)**.5)**2/4
            ref=quad(f,0,1,points=[.51-.1,.51,.51+.1],epsabs=1e-22)[0]
            self.assertLessEqual(c['integral']['lower'],ref+1e-22)
            self.assertGreaterEqual(c['integral']['upper'],ref-1e-22)
            if y<2: self.assertEqual(c['integral_status'],'fail')

    def test_limits_and_propagation(self):
        c=certify([0,0,2,1,0,0],[0,0,9.81],1,1e-12,obstacle=(0,0,0),
                  margins=(.1,0),max_subdivisions=0)
        self.assertEqual(c['integral_status'],'uncertified')
        e=interval_difference([0]*6,[0,0,0,1,0,0],2)
        self.assertGreaterEqual(e,2)
        self.assertEqual(flow([0]*6,[0,0,9.81],1),[0]*6)

    def test_scaling_duration_and_lifted_defect(self):
        B=F(1,10**11); dt=F(2,3); N=6; scale=10000
        self.assertEqual((B/dt)*dt,B)
        self.assertEqual((B*scale)/scale,B)
        self.assertEqual(F(3,N)*N,3)
        c=certify([0,0,10,0,0,0],[0,0,9.81],dt,B)
        t=tightening(c,B,0,dt,(.1,.1),accumulator_increment=0,native_integral=B*2)
        self.assertEqual(F(t['accumulator_defect']['lower_rational']),B*2)
        # Equal maxima do not imply per-interval agreement.
        a=[1.,0.]; b=[0.,1.]
        self.assertEqual(max(a)-max(b),0.)
        self.assertEqual(max(map(abs,per_interval_errors(a,b))),1.)

    def test_saved_n6_misses_density_but_clears_physical_obstacle(self):
        fixture=json.loads((Path(__file__).parent/'fixtures/free_ddp_ctcs_n6.json').read_text())
        row=fixture['row']; node=fixture['nodes'][3]
        x=[float(node[s]) for s in ('px','py','pz','vx','vy','vz')]
        u=[float(node[s]) for s in ('ux','uy','uz')]; dt=float(node['theta'])
        obs=[float(row[s]) for s in ('obs_x','obs_y','obs_z')]
        margins=(float(row['tighten_obs']),float(row['tighten_glide']))
        for steps in (4,8):
            self.assertEqual(numerical_integral(x,u,dt,steps,margins,obs,2.,1.),0.)
        c=certify(x,u,dt,float(row['eps']),margins=margins,obstacle=obs)
        self.assertEqual(c['integral_status'],'fail'); self.assertEqual(c['physical_status'],'pass')
        ref,_=reference_integral(x,u,dt,margins,obs,2.,1.)
        self.assertAlmostEqual(ref/1e-10,3.60518082,places=6)
        self.assertLess(c['integral']['lower'],ref); self.assertGreater(c['integral']['upper'],ref)

    def test_input_and_export_rounding(self):
        self.assertEqual(certify_input([0,0,9.81])['status'],'pass')
        self.assertEqual(certify_input([20,0,9.81])['status'],'fail')
        self.assertLess(rounding_radius('0.04'),F(1,10**12))
        self.assertEqual(rounding_radius('0'),0)

    def test_rounding_envelope(self):
        error,E=rounding_flow_error([0]*6,[0,0,9.81],1,[F(0)]*6,[F(1,100)]*3,F(0))
        self.assertGreaterEqual(E,F(1,200))
        self.assertEqual(error[3],F(1,100))


if __name__=='__main__': unittest.main()
