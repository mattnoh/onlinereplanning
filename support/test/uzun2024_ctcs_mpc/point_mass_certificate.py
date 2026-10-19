"""Validated nominal ZOH point-mass audit. All enclosure arithmetic is rational.

Bounds use power-to-Bernstein conversion and dyadic square-root brackets.
Floating point appears only in outward-rounded display fields. No optimizer.
"""
from fractions import Fraction as F
from math import comb, isqrt, nextafter, inf
import heapq


def q(x):
    return x if isinstance(x, F) else F(float(x))


def sqrt_bounds(x, bits=90):
    x = max(F(0), x)
    n = isqrt((x.numerator << (2*bits)) // x.denominator)
    lo = F(n, 1 << bits)
    return lo, lo if lo*lo == x else F(n+1, 1 << bits)


def enclosure(lo, hi):
    return dict(lower=nextafter(float(lo), -inf) if lo else 0.,
                upper=nextafter(float(hi), inf) if hi else 0.,
                lower_rational=str(lo), upper_rational=str(hi))


def polynomial_bounds(c, a, b):
    """Convex hull of exact Bernstein coefficients on [a,b]."""
    n = len(c)-1
    power = [sum(c[j]*comb(j,i)*a**(j-i)*(b-a)**i for j in range(i,n+1))
             for i in range(n+1)]
    bernstein = [sum(power[j]*F(comb(i,j),comb(n,j)) for j in range(i+1))
                 for i in range(n+1)]
    return min(bernstein), max(bernstein)


def square_sum(polys):
    out = [F(0)]*(2*max(map(len,polys))-1)
    for c in polys:
        for i, a in enumerate(c):
            for j, b in enumerate(c):
                out[i+j] += a*b
    return out


def flow(x, u, dt, gravity=9.81):
    x, u, dt = list(map(q,x)), list(map(q,u)), q(dt)
    a = [u[j]-(q(gravity) if j==2 else 0) for j in range(3)]
    return [x[j]+dt*x[j+3]+dt*dt*a[j]/2 for j in range(3)]+[x[j+3]+dt*a[j] for j in range(3)]


def certify(x, u, duration, budget, margins=(0.,0.), obstacle=(2.85355,2.14645,5.5),
            radius=2., glide_tan=1., position_error=0, duration_error=0,
            max_subdivisions=100000, max_depth=60, relative_width=F(1,10000)):
    """Enclose physical density integral and strict path residuals separately.

    position_error bounds the Euclidean position error at matching normalized
    time (for CSV rounding). duration_error bounds the integration measure.
    Endpoints and all intermediate bounds refer to exact nominal dynamics.
    """
    x, u = list(map(q,x)), list(map(q,u))
    dt, B, R = q(duration), q(budget), q(radius)
    pe, de = q(position_error), q(duration_error)
    if dt<=de or B<=0 or R<=0 or pe<0 or de<0:
        raise ValueError('positive duration, budget, radius and nonnegative errors required')
    m = list(map(q,margins)); tangent=q(glide_tan)
    acc=[u[j]-(q(9.81) if j==2 else 0) for j in range(3)]
    # Parameterize on [0,1]; coefficients and interval endpoints remain exact.
    p=[[x[j],dt*x[j+3],dt*dt*acc[j]/2] for j in range(3)]
    dist=square_sum([[p[j][0]-q(obstacle[j])]+p[j][1:] for j in range(3)])
    radial=square_sum(p[:2]); radial[0]+=q(1e-12)
    z=[tangent*c for c in p[2]]
    glide_error=sqrt_bounds(1+tangent*tangent)[1]*pe

    def box(a,b,depth):
        d0,d1=polynomial_bounds(dist,a,b)
        r0,r1=polynomial_bounds(radial,a,b)
        z0,z1=polynomial_bounds(z,a,b)
        obs=(R-sqrt_bounds(d1)[1]-pe, R-sqrt_bounds(d0)[0]+pe)
        glide=(sqrt_bounds(r0)[0]-z1-glide_error, sqrt_bounds(r1)[1]-z0+glide_error)
        low=sum(max(F(0),v[0]+t)**2 for v,t in zip((obs,glide),m))/R**2
        high=sum(max(F(0),v[1]+t)**2 for v,t in zip((obs,glide),m))/R**2
        return ((b-a)*(dt-de)*low,(b-a)*(dt+de)*high,obs,glide,a,b,depth)

    first=box(F(0),F(1),0); heap=[]; serial=0
    heapq.heappush(heap, (-(first[1]-first[0]),serial,first))
    lo,hi=first[:2]; subdivisions=0
    def uncertain(r): return int(any(v[1]>0 for v in r[2:4]))
    def violated(r): return int(any(v[0]>0 for v in r[2:4]))
    uncertain_count, violated_count = uncertain(first), violated(first)
    while True:
        physical_pass=uncertain_count==0
        physical_fail=violated_count>0
        decision='pass' if hi<=B else 'fail' if lo>B else 'uncertified'
        if ((decision!='uncertified' and (physical_pass or physical_fail))
                or hi-lo<=relative_width*B or subdivisions>=max_subdivisions):
            break
        _,_,r=heapq.heappop(heap)
        if r[6]>=max_depth:
            heapq.heappush(heap, (-(r[1]-r[0]),serial,r)); break
        a,b=r[4:6]; mid=(a+b)/2
        children=[box(a,mid,r[6]+1),box(mid,b,r[6]+1)]
        lo+=sum(c[0] for c in children)-r[0]; hi+=sum(c[1] for c in children)-r[1]
        for child in children:
            serial+=1
            heapq.heappush(heap, (-(child[1]-child[0]),serial,child))
        uncertain_count+=sum(uncertain(c) for c in children)-uncertain(r)
        violated_count+=sum(violated(c) for c in children)-violated(r)
        subdivisions+=1
    residuals=[enclosure(max(r[j][0] for _,_,r in heap),max(r[j][1] for _,_,r in heap)) for j in (2,3)]
    end=flow(x,u,dt)
    speed=max(sqrt_bounds(sum(v*v for v in state[3:]))[1] for state in (x,end))
    return dict(integral=enclosure(lo,hi), integral_status=decision,
                physical_status='pass' if physical_pass else 'fail' if physical_fail else 'uncertified',
                physical_residual_max_m=residuals, speed_upper_m_s=enclosure(speed,speed),
                subdivisions=subdivisions, max_depth=max(r[6] for _,_,r in heap),
                enclosure_width_target_met=hi-lo<=relative_width*B,
                stopped_at_limit=subdivisions>=max_subdivisions or any(r[6]>=max_depth for _,_,r in heap))


def certify_input(u, maximum=1.5*9.81, cone_tangent=0.36397023426620234):
    """Strict ZOH input norm, cone and nonnegative vertical acceleration."""
    u=list(map(q,u)); norm=sqrt_bounds(sum(v*v for v in u))
    radial=sqrt_bounds(u[0]*u[0]+u[1]*u[1])
    residuals=[(norm[0]-q(maximum),norm[1]-q(maximum)),
               (radial[0]-q(cone_tangent)*u[2],radial[1]-q(cone_tangent)*u[2]),(-u[2],-u[2])]
    status='pass' if all(hi<=0 for lo,hi in residuals) else 'fail' if any(lo>0 for lo,hi in residuals) else 'uncertified'
    return {'status':status,'residuals':dict(zip(('norm_excess_m_s2','cone_excess_m_s2','negative_vertical_m_s2'),
                                             (enclosure(lo,hi) for lo,hi in residuals)))}


def interval_difference(local, propagated, duration):
    """Exact endpoint hull for affine position difference under identical inputs."""
    dt=q(duration)
    p0=[q(propagated[j])-q(local[j]) for j in range(3)]
    p1=[p0[j]+dt*(q(propagated[j+3])-q(local[j+3])) for j in range(3)]
    return max(sqrt_bounds(sum(v*v for v in p))[1] for p in (p0,p1))


def tightening(cert, budget, computed_integral, duration, margins, position_difference=0,
               accumulator_increment=None, native_integral=None, radius=2., glide_tan=1., duration_lower_bound=None):
    """Sufficient margins; local quadrature error and propagation error counted once.

    For lifted rows native_integral is the independently reintegrated numerical
    map. If absent, no lifted tightening certificate is issued.
    """
    B,dt,R=map(q,(budget,duration,radius)); Q=q(computed_integral)
    if accumulator_increment is not None:
        if native_integral is None:
            return {'status':'uncertified','reason':'native accumulator reintegration unavailable'}
        increment=q(accumulator_increment)
        row_residual=max(F(0),increment-B)
        defect=abs(increment-q(native_integral)); Q=q(native_integral)
    else:
        row_residual=max(F(0),Q-B); defect=F(0)
    lo=F(cert['integral']['lower_rational']); hi=F(cert['integral']['upper_rational'])
    eta=max(abs(lo-Q),abs(hi-Q))
    effective=B+row_residual+defect+eta
    dt_min=dt if duration_lower_bound is None else q(duration_lower_bound)
    if not 0<dt_min<=dt: raise ValueError('invalid duration lower bound')
    speed=F(cert['speed_upper_m_s']['upper_rational'])
    tangent=q(glide_tan)
    factors=[F(1),sqrt_bounds(1+tangent*tangent)[1]]
    rows=[]
    for margin,factor in zip(margins,factors):
        L=factor*speed/R; E=factor*q(position_difference)/R
        condition=effective<=L*L*dt_min**3/4
        if condition and L>0:
            # Upper dyadic cube-root bracket via integer binary search.
            target=4*L*effective; scale=1<<90
            n=(target.numerator*scale**3)//target.denominator
            a,b=0,1<<((n.bit_length()+2)//3)
            while a+1<b:
                c=(a+b)//2
                if c**3<=n: a=c
                else: b=c
            sufficient=R*(F(b,scale)+E)
        else:
            sufficient=R*(sqrt_bounds(effective/dt)[1]+L*dt+E)
        reserve=q(margin)-sufficient
        rows.append(dict(configured_m=float(margin), sufficient_upper_m=enclosure(sufficient,sufficient),
                         reserve_lower_m=enclosure(reserve,reserve), small_budget_condition=condition,
                         derivative_bound=enclosure(L,L), propagated_error_m=enclosure(R*E,R*E)))
    return dict(duration_lower_bound=enclosure(dt_min,dt_min), status='pass' if all(q(m)-F(r['sufficient_upper_m']['upper_rational'])>=0 for m,r in zip(margins,rows)) else 'fail',
                effective_budget=enclosure(effective,effective), evaluation_error=enclosure(eta,eta),
                row_residual=enclosure(row_residual,row_residual), accumulator_defect=enclosure(defect,defect), constraints=rows)


def rounding_radius(text):
    """Enclose 12-significant-digit export rounding and decimal-to-binary error."""
    from decimal import Decimal
    value=Decimal(text)
    # The historical writer uses std::setprecision(12), defaultfloat:
    # twelve significant digits, with trailing zeroes omitted. Zero is exact.
    if not value: return F(0)
    return F(Decimal(1).scaleb(value.adjusted()-11))/2 + abs(F(value)-q(float(value)))


def rounding_flow_error(x,u,dt,dx,du,ddt):
    """Componentwise error recursion at matching normalized time, including dt."""
    x,u,dt=list(map(q,x)),list(map(q,u)),q(dt)
    acc=[u[j]-(q(9.81) if j==2 else 0) for j in range(3)]
    dp=[dx[j]+dt*dx[j+3]+ddt*(abs(x[j+3])+dx[j+3])+dt*dt*du[j]/2
        +(2*dt*ddt+ddt*ddt)*(abs(acc[j])+du[j])/2 for j in range(3)]
    dv=[dx[j+3]+dt*du[j]+ddt*(abs(acc[j])+du[j]) for j in range(3)]
    return dp+dv, sqrt_bounds(sum(v*v for v in dp))[1]
