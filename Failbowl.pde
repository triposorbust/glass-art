PVector rand_ortho(PVector orig) {
  float x = random(-1,1);
  float y = random(-1,1);
  float z = (-orig.x*x -orig.y*y)/orig.z;
  
  PVector rslt = new PVector(x,y,z);
  rslt.normalize();

  return rslt;
}

// DC element
class DCe {
  PVector n,r;
  float len,rad;
  DCe (PVector _n, PVector _r, float _len, float _rad) {
    n=_n; len=_len;
    r=_r; rad=_rad;
  }
}

// DC (point) Orbit
class DCo {
  PVector[] pt;
  PVector loc;
  DCe spec;
  
  DCo (DCe _spec, PVector _loc, int npts) {
    pt = new PVector[npts];
    spec = _spec;
    loc = _loc;
    
    PVector ax = spec.n.cross(spec.r);
    for (int i=0; i<npts; ++i) {
      float th = TWO_PI * float(i) / float(npts-1);
      pt[i] = new PVector(
        loc.x + spec.rad * (spec.r.x * cos(th) + ax.x * sin(th)),
        loc.y + spec.rad * (spec.r.y * cos(th) + ax.y * sin(th)),
        loc.z + spec.rad * (spec.r.z * cos(th) + ax.z * sin(th)));
    }
  }
}

// DC composite form
class DCx {
  PShape memo;
  DCe[] elem;
  PVector p0;
  int npts;
  
  DCx(DCe[] _elem, PVector _p0, int _npts) {
    elem = _elem;
    npts = _npts;
    p0 = _p0;
  }
  
  void display () {
    if (null != memo) {
      shape(memo);
    } else {
      PVector curr = new PVector(p0.x, p0.y, p0.z);
      for (int i=0; i<elem.length-1; ++i) {
        if (elem[i].rad == 0) { break; }
        PVector next = new PVector(curr.x + elem[i].n.x * elem[i].len,
                                   curr.y + elem[i].n.y * elem[i].len,
                                   curr.z + elem[i].n.z * elem[i].len);
        
        int j=i+1;
        DCo oi = new DCo(elem[i], curr, npts);
        DCo oj = new DCo(elem[j], next, npts);
        
        beginShape(QUAD_STRIP);
        for (int k=0; k<npts; ++k) {
          vertex(oi.pt[k].x, oi.pt[k].y, oi.pt[k].z);
          vertex(oj.pt[k].x, oj.pt[k].y, oj.pt[k].z);
        }
        endShape();
        
        curr.x = next.x;
        curr.y = next.y;
        curr.z = next.z;
      }
    }
  }
  
  void memoize() {
    if (null != memo) {
      memo = createShape(GROUP);
      
      PVector curr = new PVector(p0.x, p0.y, p0.z);
      for (int i=0; i<elem.length-1; ++i) {
        if (elem[i].rad == 0) { break; }
        PVector next = new PVector(curr.x + elem[i].n.x * elem[i].len,
                                   curr.y + elem[i].n.y * elem[i].len,
                                   curr.z + elem[i].n.z * elem[i].len);
     
        int j=i+1;
        DCo oi = new DCo(elem[i], curr, npts);
        DCo oj = new DCo(elem[j], next, npts);
      
        PShape s = createShape();
        s.beginShape(QUAD_STRIP);
        for (int k=0; k<npts; ++k) {
          s.vertex(oi.pt[k].x, oi.pt[k].y, oi.pt[k].z);
          s.vertex(oj.pt[k].x, oj.pt[k].y, oj.pt[k].z);
        }
        s.endShape();
        memo.addChild(s);
        
        curr.x = next.x;
        curr.y = next.y;
        curr.z = next.z;
      }
    }
  }
}

class DC_Series {
  float[] len;
  float[] rad;
  DCx dcx;

  int offset;
  float posn;
  
  /* for N waypoints + N reference pts,
     need N-1 segment lengths and radii
     last len/rad should be 0.0 and 0.0 */
  DC_Series (PVector[] pt, PVector[] ref, float[] _len, float[] _rad) {
    posn = 0.0;
    
    DCe[] elem = new DCe[pt.length-1];
    for (int i=0; i<elem.length; ++i) {
      elem[i] = new DCe(PVector.sub(pt[i+1],pt[i]),
                        PVector.sub(ref[i], pt[i]),
                        0.0,0.0);
    }
    dcx = new DCx(elem, pt[0], 25);
    offset = elem.length-1;
    
    len=_len;
    rad=_rad;
  }
  
  void step(float dposn) {
    if (offset > 0) {
      if (posn <= 1) {
        posn += dposn;
        for (int i=0; (i+offset)<dcx.elem.length; ++i) {
          // i element get i+offset spec
          int j = i+offset;
          dcx.elem[i].rad = rad[j] + posn * (rad[j-1]-rad[j]);
          dcx.elem[i].len = len[j] + posn * (len[j-1]-len[j]);
        }
      } else {
        posn = 0.0;
        offset--;
        if (offset < 0) {
          for (int i=0; i<dcx.elem.length; ++i) {
            dcx.elem[i].rad = rad[i];
            dcx.elem[i].len = len[i];
          }
          dcx.memoize();
        }
      }
    }
  }

  void display() {
    dcx.display();
  }
}

class DC_Cone {
  DC_Series dcs;
  color clr;
  
  DC_Cone (PVector loc, PVector nrm, float _len, float _rad) {
    PVector _ref = rand_ortho(nrm);
    PVector[] pt  = {
      PVector.add(loc,PVector.mult(nrm,0.0)),
      PVector.add(loc,PVector.mult(nrm,1.0)),
      PVector.add(loc,PVector.mult(nrm,2.0)) };
    PVector[] ref = { 
      PVector.add(pt[0], _ref),
      PVector.add(pt[1], _ref),
      PVector.add(pt[2], _ref) };
    float[] len = { _len,0. };
    float[] rad = { _rad,0. };
    
    clr = color(
    random(230)+25,
    random(230)+25,
    random(230)+25,
    205);
    dcs = new DC_Series(pt,ref,len,rad);
  }
  
  void step(float dposn) {
    dcs.step(abs(dposn));
  }
  
  void display() {
    fill(clr);
    noStroke(); //stroke(clr);
    dcs.display();
  }
}

class DC_Bulb {
  DC_Series dcs;
  color clr;
  
  DC_Bulb (PVector loc, PVector nrm,
           float tot_len, float init_rad,
           float bulb_ctr_rel, float bulb_rad,
           float jit_mf) {
    float bulb_ctr = bulb_ctr_rel * tot_len;
    float seg_len = bulb_rad/5.0;
    tot_len = max(bulb_rad + bulb_ctr + 2*seg_len, tot_len);
    
    int n_seg = (int) (round(tot_len/seg_len) + 1.0);
    float[] rad = new float[n_seg];
    float[] len = new float[n_seg];

    for (int i=0; i<n_seg; ++i) {
      float seg_ctr = seg_len * ((float) i + 0.5);
      float dctr = abs(seg_ctr - bulb_ctr);

      if (dctr < bulb_rad) {
        rad[i] = max(
          sqrt(sq(bulb_rad) - sq(dctr)),
          //bulb_rad*sin( acos(dctr/bulb_rad) ),
          init_rad);
      } else if (i >= n_seg-3) {
        rad[i] = pow(5,n_seg-3-i);
      } else {
        rad[i] = init_rad;
      }
      len[i] = seg_len;
    }
    
    len[n_seg-1] = 0.0;
    rad[n_seg-1] = 0.0;
    
    PVector _ref = rand_ortho(nrm);
    PVector[] pt  = new PVector[n_seg+1];
    PVector[] ref = new PVector[n_seg+1];
    
    for (int i=0; i<=n_seg; ++i) {
      PVector seg_loc =
        PVector.add(loc,
          PVector.mult(nrm, (float) i * seg_len));
      pt[i] = seg_loc;
      ref[i] = PVector.add(seg_loc,_ref);
    }
    
    dcs = new DC_Series(pt, ref, len, rad);
    clr = color(
    random(230)+25,
    random(230)+25,
    random(230)+25,
    185);
  }
  
  void step(float dposn) {
    dcs.step(abs(dposn));
  }
  
  void display() {
    fill(clr);
    // stroke(clr);
    noStroke();
    dcs.display();
  }
}

class DC_Helix {
  DC_Series dcs;
  color clr;
  
  DC_Helix (PVector nrm, PVector loc,
            float len, float rh, float ri,
            int nstep, int nturn) {
    float dtheta = TWO_PI * ((float) nturn / (float) nstep);
    clr=color(
    random(230)+25,
    random(230)+25,
    random(230)+25,
    205);

    PVector[] pt  = new PVector[nstep];
    PVector[] ref = new PVector[nstep];
    
    float[] slen = new float[nstep-1];
    float[] srad = new float[nstep-1];
    
    PVector v0 = rand_ortho(nrm);
    PVector v1;
    if (random(0,1) <= 0.5) v1 = v0.cross(nrm);
    else v1 = nrm.cross(v0);

    for (int i=0; i<nstep; ++i) {
      float rel = (float)i / (float)nstep;
      float r1 = lerp(rh,0,rel);
      float theta = dtheta * (float)i;
      
      ref[i] = new PVector(loc.x + nrm.x*rel*len,
                           loc.y + nrm.y*rel*len,
                           loc.z + nrm.z*rel*len);
      pt[i]  = new PVector(ref[i].x + r1*(v0.x*cos(theta) + v1.x*sin(theta)),
                           ref[i].y + r1*(v0.y*cos(theta) + v1.y*sin(theta)),
                           ref[i].z + r1*(v0.z*cos(theta) + v1.z*sin(theta)));
      
      if (i > 0) {                     
        srad[i-1] = lerp(ri,0,(float) (i-1) / (float) (nstep-1));
        slen[i-1] = len / (float)(nstep-1);
      }
    }
    
    dcs = new DC_Series(pt, ref, slen, srad);
  }
  
  void step(float dposn) {
    dcs.step(abs(dposn));
  }
  
  void display() {
    fill(clr);
    noStroke(); // stroke(clr);
    dcs.display();
  }
}

DC_Bulb[] belem;
DC_Cone[] celem;
DC_Helix[] helem;

boolean is_init=false;
boolean on_hold=true;

void do_init() {
  if (is_init) {
    do_term();
  }
  PVector loc = new PVector(width/2,height/2,-50);
  
  int nbelem = (int) random(12)+7;
  belem = new DC_Bulb[nbelem];
  for (int i=0; i<belem.length; ++i) {
    PVector nrm = new PVector(random(-1,1),random(-1,1),random(-1,1));
    nrm.normalize();

    belem[i] = new DC_Bulb(loc,nrm,
                           75+randomGaussian()*15,
                           0.45+randomGaussian()*0.05,
                           0.975,
                           5+randomGaussian()*0.5,
                           0.0 /* deprecated */);
  }
  
  int ncelem = (int) random(25)+15;
  celem = new DC_Cone[ncelem];
  for (int i=0; i<celem.length; ++i) {
    PVector nrm = new PVector(random(-1,1),random(-1,1),random(-1,1));
    nrm.normalize();
    
    celem[i] = new DC_Cone(loc,nrm,
                           85.+ 15 * randomGaussian(),
                           2.5 + 1.5 * randomGaussian());
  }
  
  int nhelem = (int) random(40)+17;
  helem = new DC_Helix[nhelem];
  for (int i=0; i<helem.length; ++i) {
    PVector nrm = new PVector(random(-1,1),random(-1,1),random(-1,1));
    nrm.normalize();
    int nturn = (int) random(5)+4;
    
    helem[i] = new DC_Helix(nrm, loc,
                               115 + 22.5  * randomGaussian(),
                               4.0 + 0.75  * randomGaussian(),
                               1.5 + 0.45  * randomGaussian(),
                               25+nturn*15,nturn);
  }
  
  is_init = true;
  on_hold = false;
}

void do_term() {
  if (is_init) {
  belem = null;
  celem = null;
  helem = null;
  
  is_init = false;
  on_hold = true;
  }
}

void keyPressed() {
  if (key == 13 || key == 10) {
    do_init();
  }
  if (key == 32) {
    on_hold = !on_hold;
  }
}

void settings() {
  size(500,500,P3D);
}

void setup() { }

void draw() {
  camera(mouseX, mouseY, (height/2)/tan(PI/3), width/2, height/2, 0, 0, 1, 0);
  background(10);
  
  if (is_init) {
  ambientLight(125,125,125);
  directionalLight(255,255,255,0,1,0);
  lightSpecular(255,255,255);
  shininess(1000.0);
  
  for (int i=0; i<belem.length; ++i) {
    if (!on_hold) {
      belem[i].step(pow(10,random(-0.75,-0.15)));
    }
    belem[i].display();
  }
  
  for (int i=0; i<celem.length; ++i) {
    if (!on_hold) {
      celem[i].step(pow(10,random(-4.5,-1.5)));
    }
    celem[i].display();
  }

  for (int i=0; i<helem.length; ++i) {
    if (!on_hold) {
      helem[i].step(pow(10,random(-1,0)));
    }
    helem[i].display();
  }
  }
}
