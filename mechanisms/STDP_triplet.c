/* Created by Language version: 6.2.0 */
/* VECTORIZED */
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "scoplib_ansi.h"
#undef PI
#define nil 0
#include "md1redef.h"
#include "section.h"
#include "nrniv_mf.h"
#include "md2redef.h"
 
#if METHOD3
extern int _method3;
#endif

#if !NRNGPU
#undef exp
#define exp hoc_Exp
extern double hoc_Exp(double);
#endif
 
#define _threadargscomma_ _p, _ppvar, _thread, _nt,
#define _threadargs_ _p, _ppvar, _thread, _nt
 
#define _threadargsprotocomma_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt,
#define _threadargsproto_ double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt
 	/*SUPPRESS 761*/
	/*SUPPRESS 762*/
	/*SUPPRESS 763*/
	/*SUPPRESS 765*/
	 extern double *getarg();
 /* Thread safe. No static _p or _ppvar. */
 
#define t _nt->_t
#define dt _nt->_dt
#define e _p[0]
#define tau _p[1]
#define factor _p[2]
#define gw _p[3]
#define A2mais _p[4]
#define A2menos _p[5]
#define A3mais _p[6]
#define A3menos _p[7]
#define Taumais _p[8]
#define Taumenos _p[9]
#define Tauy _p[10]
#define Taux _p[11]
#define i _p[12]
#define t_last_pre _p[13]
#define t_last_post _p[14]
#define flag_pre _p[15]
#define flag_post _p[16]
#define flag1_pre _p[17]
#define flag1_post _p[18]
#define g _p[19]
#define R1 _p[20]
#define R2 _p[21]
#define O1 _p[22]
#define O2 _p[23]
#define peso _p[24]
#define tpost _p[25]
#define Dg _p[26]
#define DR1 _p[27]
#define DR2 _p[28]
#define DO1 _p[29]
#define DO2 _p[30]
#define Dpeso _p[31]
#define v _p[32]
#define _g _p[33]
#define _tsav _p[34]
#define _nd_area  *_ppvar[0]._pval
 
#if MAC
#if !defined(v)
#define v _mlhv
#endif
#if !defined(h)
#define h _mlhh
#endif
#endif
 
#if defined(__cplusplus)
extern "C" {
#endif
 static int hoc_nrnpointerindex =  -1;
 static Datum* _extcall_thread;
 static Prop* _extcall_prop;
 /* external NEURON variables */
 /* declaration of user functions */
 static int _mechtype;
extern void _nrn_cacheloop_reg(int, int);
extern void hoc_register_prop_size(int, int, int);
extern void hoc_register_limits(int, HocParmLimits*);
extern void hoc_register_units(int, HocParmUnits*);
extern void nrn_promote(Prop*, int, int);
extern Memb_func* memb_func;
 extern Prop* nrn_point_prop_;
 static int _pointtype;
 static void* _hoc_create_pnt(_ho) Object* _ho; { void* create_point_process();
 return create_point_process(_pointtype, _ho);
}
 static void _hoc_destroy_pnt();
 static double _hoc_loc_pnt(_vptr) void* _vptr; {double loc_point_process();
 return loc_point_process(_pointtype, _vptr);
}
 static double _hoc_has_loc(_vptr) void* _vptr; {double has_loc_point();
 return has_loc_point(_vptr);
}
 static double _hoc_get_loc_pnt(_vptr)void* _vptr; {
 double get_loc_point_process(); return (get_loc_point_process(_vptr));
}
 extern void _nrn_setdata_reg(int, void(*)(Prop*));
 static void _setdata(Prop* _prop) {
 _extcall_prop = _prop;
 }
 static void _hoc_setdata(void* _vptr) { Prop* _prop;
 _prop = ((Point_process*)_vptr)->_prop;
   _setdata(_prop);
 }
 /* connect user functions to hoc names */
 static VoidFunc hoc_intfunc[] = {
 0,0
};
 static Member_func _member_func[] = {
 "loc", _hoc_loc_pnt,
 "has_loc", _hoc_has_loc,
 "get_loc", _hoc_get_loc_pnt,
 0, 0
};
 /* declare global and static user variables */
#define latency latency_ExpSynSTDP_triplet
 double latency = 0;
 /* some parameters have upper and lower limits */
 static HocParmLimits _hoc_parm_limits[] = {
 0,0,0
};
 static HocParmUnits _hoc_parm_units[] = {
 "e", "mV",
 "tau", "ms",
 "g", "uS",
 "i", "nA",
 "t_last_pre", "ms",
 "t_last_post", "ms",
 0,0
};
 static double O20 = 0;
 static double O10 = 0;
 static double R20 = 0;
 static double R10 = 0;
 static double delta_t = 0.01;
 static double g0 = 0;
 static double peso0 = 0;
 /* connect global user variables to hoc */
 static DoubScal hoc_scdoub[] = {
 "latency_ExpSynSTDP_triplet", &latency_ExpSynSTDP_triplet,
 0,0
};
 static DoubVec hoc_vdoub[] = {
 0,0,0
};
 static double _sav_indep;
 static void nrn_alloc(Prop*);
static void  nrn_init(_NrnThread*, _Memb_list*, int);
static void nrn_state(_NrnThread*, _Memb_list*, int);
 static void nrn_cur(_NrnThread*, _Memb_list*, int);
static void  nrn_jacob(_NrnThread*, _Memb_list*, int);
 
#define _watch_array _ppvar + 3 
 
#define _fnc_index 5
 static void _hoc_destroy_pnt(_vptr) void* _vptr; {
   Prop* _prop = ((Point_process*)_vptr)->_prop;
   if (_prop) { _nrn_free_watch(_prop->dparam, 3, 2);}
   if (_prop) { _nrn_free_fornetcon(&(_prop->dparam[_fnc_index]._pvoid));}
   destroy_point_process(_vptr);
}
 
static int _ode_count(int);
static void _ode_map(int, double**, double**, double*, Datum*, double*, int);
static void _ode_spec(_NrnThread*, _Memb_list*, int);
static void _ode_matsol(_NrnThread*, _Memb_list*, int);
 
#define _cvode_ieq _ppvar[6]._i
 /* connect range variables in _p that hoc is supposed to know about */
 static const char *_mechanism[] = {
 "6.2.0",
"ExpSynSTDP_triplet",
 "e",
 "tau",
 "factor",
 "gw",
 "A2mais",
 "A2menos",
 "A3mais",
 "A3menos",
 "Taumais",
 "Taumenos",
 "Tauy",
 "Taux",
 0,
 "i",
 "t_last_pre",
 "t_last_post",
 "flag_pre",
 "flag_post",
 "flag1_pre",
 "flag1_post",
 0,
 "g",
 "R1",
 "R2",
 "O1",
 "O2",
 "peso",
 0,
 0};
 
extern Prop* need_memb(Symbol*);

static void nrn_alloc(Prop* _prop) {
	Prop *prop_ion;
	double *_p; Datum *_ppvar;
  if (nrn_point_prop_) {
	_prop->_alloc_seq = nrn_point_prop_->_alloc_seq;
	_p = nrn_point_prop_->param;
	_ppvar = nrn_point_prop_->dparam;
 }else{
 	_p = nrn_prop_data_alloc(_mechtype, 35, _prop);
 	/*initialize range parameters*/
 	e = 0;
 	tau = 10;
 	factor = 1;
 	gw = 0.01;
 	A2mais = 0.0046;
 	A2menos = 0.003;
 	A3mais = 0.0091;
 	A3menos = 7.5e-009;
 	Taumais = 16.8;
 	Taumenos = 33.7;
 	Tauy = 47;
 	Taux = 575;
  }
 	_prop->param = _p;
 	_prop->param_size = 35;
  if (!nrn_point_prop_) {
 	_ppvar = nrn_prop_datum_alloc(_mechtype, 7, _prop);
  }
 	_prop->dparam = _ppvar;
 	/*connect ionic variables to this model*/
 
}
 static void _initlists();
  /* some states have an absolute tolerance */
 static Symbol** _atollist;
 static HocStateTolerance _hoc_state_tol[] = {
 0,0
};
 
#define _tqitem &(_ppvar[2]._pvoid)
 static void _net_receive(Point_process*, double*, double);
 extern int _nrn_netcon_args(void*, double***);
 static void _net_init(Point_process*, double*, double);
 extern Symbol* hoc_lookup(const char*);
extern void _nrn_thread_reg(int, int, void(*f)(Datum*));
extern void _nrn_thread_table_reg(int, void(*)(double*, Datum*, Datum*, _NrnThread*, int));
extern void hoc_register_tolerance(int, HocStateTolerance*, Symbol***);
extern void _cvode_abstol( Symbol**, double*, int);

 void _STDP_triplet_reg() {
	int _vectorized = 1;
  _initlists();
 	_pointtype = point_register_mech(_mechanism,
	 nrn_alloc,nrn_cur, nrn_jacob, nrn_state, nrn_init,
	 hoc_nrnpointerindex, 1,
	 _hoc_create_pnt, _hoc_destroy_pnt, _member_func);
 _mechtype = nrn_get_mechtype(_mechanism[1]);
     _nrn_setdata_reg(_mechtype, _setdata);
  hoc_register_prop_size(_mechtype, 35, 7);
 	hoc_register_cvode(_mechtype, _ode_count, _ode_map, _ode_spec, _ode_matsol);
 	hoc_register_tolerance(_mechtype, _hoc_state_tol, &_atollist);
 pnt_receive[_mechtype] = _net_receive;
 pnt_receive_init[_mechtype] = _net_init;
 pnt_receive_size[_mechtype] = 3;
 add_nrn_fornetcons(_mechtype, _fnc_index);
 	hoc_register_var(hoc_scdoub, hoc_vdoub, hoc_intfunc);
 	ivoc_help("help ?1 ExpSynSTDP_triplet C:/cygwin64/home/Fabio/Javier_2017/Superscript_2017/super/mechanisms/STDP_triplet.mod\n");
 hoc_register_limits(_mechtype, _hoc_parm_limits);
 hoc_register_units(_mechtype, _hoc_parm_units);
 }
static int _reset;
static char *modelname = "Stream-Specific Spike Timing Dependent Plasticity";

static int error;
static int _ninits = 0;
static int _match_recurse=1;
static void _modl_cleanup(){ _match_recurse=1;}
 
static int _ode_spec1(_threadargsproto_);
/*static int _ode_matsol1(_threadargsproto_);*/
 static int _slist1[9], _dlist1[9];
 static int state(_threadargsproto_);
 
/*CVODE*/
 static int _ode_spec1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {int _reset = 0; {
   if ( flag1_pre  == 0.0 ) {
     DR1 = - R1 / Taumais / 1.0 ;
     DR2 = - R2 / Taux / 1.0 ;
     }
   if ( flag1_pre  == 1.0 ) {
     DR1 = ( - R1 + 1.0 ) / Taumais / 1.0 ;
     DR2 = ( - R2 + 1.0 ) / Taux / 1.0 ;
     flag1_pre = 0.0 ;
     }
   if ( flag1_post  == 0.0 ) {
     DO1 = - O1 / Taumenos / 1.0 ;
     DO2 = - O2 / Tauy / 1.0 ;
     }
   if ( flag1_post  == 1.0 ) {
     DO1 = ( - O1 + 1.0 ) / Taumenos / 1.0 ;
     DO2 = ( - O2 + 1.0 ) / Tauy / 1.0 ;
     flag1_post = 0.0 ;
     }
   Dg = - g / tau ;
   }
 return _reset;
}
 static int _ode_matsol1 (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
 if ( flag1_pre  == 0.0 ) {
   DR1 = DR1  / (1. - dt*( ( ( - 1.0 ) / Taumais ) / 1.0 )) ;
   DR2 = DR2  / (1. - dt*( ( ( - 1.0 ) / Taux ) / 1.0 )) ;
   }
 if ( flag1_pre  == 1.0 ) {
   DR1 = DR1  / (1. - dt*( ( ( ( - 1.0 ) ) / Taumais ) / 1.0 )) ;
   DR2 = DR2  / (1. - dt*( ( ( ( - 1.0 ) ) / Taux ) / 1.0 )) ;
   flag1_pre = 0.0 ;
   }
 if ( flag1_post  == 0.0 ) {
   DO1 = DO1  / (1. - dt*( ( ( - 1.0 ) / Taumenos ) / 1.0 )) ;
   DO2 = DO2  / (1. - dt*( ( ( - 1.0 ) / Tauy ) / 1.0 )) ;
   }
 if ( flag1_post  == 1.0 ) {
   DO1 = DO1  / (1. - dt*( ( ( ( - 1.0 ) ) / Taumenos ) / 1.0 )) ;
   DO2 = DO2  / (1. - dt*( ( ( ( - 1.0 ) ) / Tauy ) / 1.0 )) ;
   flag1_post = 0.0 ;
   }
 Dg = Dg  / (1. - dt*( ( - 1.0 ) / tau )) ;
 return 0;
}
 /*END CVODE*/
 static int state (double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) { {
   if ( flag1_pre  == 0.0 ) {
      R1 = R1 + (1. - exp(dt*(( ( - 1.0 ) / Taumais ) / 1.0)))*(- ( 0.0 ) / ( ( ( - 1.0 ) / Taumais ) / 1.0 ) - R1) ;
      R2 = R2 + (1. - exp(dt*(( ( - 1.0 ) / Taux ) / 1.0)))*(- ( 0.0 ) / ( ( ( - 1.0 ) / Taux ) / 1.0 ) - R2) ;
     }
   if ( flag1_pre  == 1.0 ) {
      R1 = R1 + (1. - exp(dt*(( ( ( - 1.0 ) ) / Taumais ) / 1.0)))*(- ( ( ( ( 1.0 ) ) / Taumais ) / 1.0 ) / ( ( ( ( - 1.0 ) ) / Taumais ) / 1.0 ) - R1) ;
      R2 = R2 + (1. - exp(dt*(( ( ( - 1.0 ) ) / Taux ) / 1.0)))*(- ( ( ( ( 1.0 ) ) / Taux ) / 1.0 ) / ( ( ( ( - 1.0 ) ) / Taux ) / 1.0 ) - R2) ;
     flag1_pre = 0.0 ;
     }
   if ( flag1_post  == 0.0 ) {
      O1 = O1 + (1. - exp(dt*(( ( - 1.0 ) / Taumenos ) / 1.0)))*(- ( 0.0 ) / ( ( ( - 1.0 ) / Taumenos ) / 1.0 ) - O1) ;
      O2 = O2 + (1. - exp(dt*(( ( - 1.0 ) / Tauy ) / 1.0)))*(- ( 0.0 ) / ( ( ( - 1.0 ) / Tauy ) / 1.0 ) - O2) ;
     }
   if ( flag1_post  == 1.0 ) {
      O1 = O1 + (1. - exp(dt*(( ( ( - 1.0 ) ) / Taumenos ) / 1.0)))*(- ( ( ( ( 1.0 ) ) / Taumenos ) / 1.0 ) / ( ( ( ( - 1.0 ) ) / Taumenos ) / 1.0 ) - O1) ;
      O2 = O2 + (1. - exp(dt*(( ( ( - 1.0 ) ) / Tauy ) / 1.0)))*(- ( ( ( ( 1.0 ) ) / Tauy ) / 1.0 ) / ( ( ( ( - 1.0 ) ) / Tauy ) / 1.0 ) - O2) ;
     flag1_post = 0.0 ;
     }
    g = g + (1. - exp(dt*(( - 1.0 ) / tau)))*(- ( 0.0 ) / ( ( - 1.0 ) / tau ) - g) ;
   }
  return 0;
}
 
static double _watch1_cond(_pnt) Point_process* _pnt; {
 	double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
	_thread= (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;
 	_p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
	v = NODEV(_pnt->node);
	return  ( v ) - ( - 20.0 ) ;
}
 
static void _net_receive (_pnt, _args, _lflag) Point_process* _pnt; double* _args; double _lflag; 
{  double* _p; Datum* _ppvar; Datum* _thread; _NrnThread* _nt;
   int _watch_rm = 0;
   _thread = (Datum*)0; _nt = (_NrnThread*)_pnt->_vnt;   _p = _pnt->_prop->param; _ppvar = _pnt->_prop->dparam;
  if (_tsav > t){ extern char* hoc_object_name(); hoc_execerror(hoc_object_name(_pnt->ob), ":Event arrived out of order. Must call ParallelContext.set_maxstep AFTER assigning minimum NetCon.delay");}
 _tsav = t;   if (_lflag == 1. ) {*(_tqitem) = 0;}
 {
   if ( _lflag  == 0.0 ) {
     g = g + _args[0] + peso ;
     if ( g < 0.0 ) {
       g = 0.0 ;
       }
     _args[2] = t ;
     _args[1] = _args[1] ;
     flag_pre = 1.0 ;
     }
   else if ( _lflag  == 2.0 ) {
     tpost = t ;
     {int _ifn1, _nfn1; double* _fnargs1, **_fnargslist1;
	_nfn1 = _nrn_netcon_args(_ppvar[_fnc_index]._pvoid, &_fnargslist1);
	for (_ifn1 = 0; _ifn1 < _nfn1; ++_ifn1) {
 	 _fnargs1 = _fnargslist1[_ifn1];
 {
       _fnargs1[1] = _fnargs1[1] ;
       flag_post = 1.0 ;
       flag_pre = 0.0 ;
       }
     	}}
 if ( flag_post  == 1.0 ) {
       flag1_post = 1.0 ;
       t_last_post = t ;
       }
     }
   else {
       _nrn_watch_activate(_watch_array, _watch1_cond, 1, _pnt, _watch_rm++, 2.0);
 }
   if ( flag_pre  == 1.0 ) {
     t_last_pre = t ;
     flag1_pre = 1.0 ;
     }
   } }
 
static void _net_init(Point_process* _pnt, double* _args, double _lflag) {
       double* _p = _pnt->_prop->param;
    Datum* _ppvar = _pnt->_prop->dparam;
    Datum* _thread = (Datum*)0;
    _NrnThread* _nt = (_NrnThread*)_pnt->_vnt;
 _args[1] = 0.0 ;
   _args[2] = 0.0 ;
   }
 
static int _ode_count(int _type){ return 9;}
 
static void _ode_spec(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
     _ode_spec1 (_p, _ppvar, _thread, _nt);
 }}
 
static void _ode_map(int _ieq, double** _pv, double** _pvdot, double* _pp, Datum* _ppd, double* _atol, int _type) { 
	double* _p; Datum* _ppvar;
 	int _i; _p = _pp; _ppvar = _ppd;
	_cvode_ieq = _ieq;
	for (_i=0; _i < 9; ++_i) {
		_pv[_i] = _pp + _slist1[_i];  _pvdot[_i] = _pp + _dlist1[_i];
		_cvode_abstol(_atollist, _atol, _i);
	}
 }
 
static void _ode_matsol(_NrnThread* _nt, _Memb_list* _ml, int _type) {
   double* _p; Datum* _ppvar; Datum* _thread;
   Node* _nd; double _v; int _iml, _cntml;
  _cntml = _ml->_nodecount;
  _thread = _ml->_thread;
  for (_iml = 0; _iml < _cntml; ++_iml) {
    _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
    _nd = _ml->_nodelist[_iml];
    v = NODEV(_nd);
 _ode_matsol1 (_p, _ppvar, _thread, _nt);
 }}

static void initmodel(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt) {
  int _i; double _save;{
  O2 = O20;
  O1 = O10;
  R2 = R20;
  R1 = R10;
  g = g0;
  peso = peso0;
 {
   g = 0.0 ;
   peso = 0.0 ;
   tpost = - 1e9 ;
   net_send ( _tqitem, (double*)0, _ppvar[1]._pvoid, t +  0.0 , 1.0 ) ;
   O1 = 0.0 ;
   O2 = 0.0 ;
   R1 = 0.0 ;
   R2 = 0.0 ;
   t_last_pre = 0.0 ;
   t_last_post = 0.0 ;
   flag_pre = 0.0 ;
   flag_post = 0.0 ;
   flag1_pre = 0.0 ;
   flag1_post = 0.0 ;
   }
 
}
}

static void nrn_init(_NrnThread* _nt, _Memb_list* _ml, int _type){
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _tsav = -1e20;
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 v = _v;
 initmodel(_p, _ppvar, _thread, _nt);
}}

static double _nrn_current(double* _p, Datum* _ppvar, Datum* _thread, _NrnThread* _nt, double _v){double _current=0.;v=_v;{ {
   if ( g < 0.0 ) {
     g = 0.0 ;
     }
   if ( flag1_pre  == 1.0 ) {
     peso = peso - factor * O1 * ( A2menos + A3menos * R2 ) ;
     }
   if ( flag1_post  == 1.0 ) {
     peso = peso + factor * R1 * ( A2mais * ( 1.0 - peso / ( 1e3 * gw ) ) + A3mais * ( 1.0 - peso / ( 1e3 * gw ) ) * O2 ) ;
     }
   i = g * ( v - e ) ;
   }
 _current += i;

} return _current;
}

static void nrn_cur(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; double _rhs, _v; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _g = _nrn_current(_p, _ppvar, _thread, _nt, _v + .001);
 	{ _rhs = _nrn_current(_p, _ppvar, _thread, _nt, _v);
 	}
 _g = (_g - _rhs)/.001;
 _g *=  1.e2/(_nd_area);
 _rhs *= 1.e2/(_nd_area);
#if CACHEVEC
  if (use_cachevec) {
	VEC_RHS(_ni[_iml]) -= _rhs;
  }else
#endif
  {
	NODERHS(_nd) -= _rhs;
  }
 
}}

static void nrn_jacob(_NrnThread* _nt, _Memb_list* _ml, int _type) {
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml];
#if CACHEVEC
  if (use_cachevec) {
	VEC_D(_ni[_iml]) += _g;
  }else
#endif
  {
     _nd = _ml->_nodelist[_iml];
	NODED(_nd) += _g;
  }
 
}}

static void nrn_state(_NrnThread* _nt, _Memb_list* _ml, int _type) {
 double _break, _save;
double* _p; Datum* _ppvar; Datum* _thread;
Node *_nd; double _v; int* _ni; int _iml, _cntml;
#if CACHEVEC
    _ni = _ml->_nodeindices;
#endif
_cntml = _ml->_nodecount;
_thread = _ml->_thread;
for (_iml = 0; _iml < _cntml; ++_iml) {
 _p = _ml->_data[_iml]; _ppvar = _ml->_pdata[_iml];
 _nd = _ml->_nodelist[_iml];
#if CACHEVEC
  if (use_cachevec) {
    _v = VEC_V(_ni[_iml]);
  }else
#endif
  {
    _nd = _ml->_nodelist[_iml];
    _v = NODEV(_nd);
  }
 _break = t + .5*dt; _save = t;
 v=_v;
{
 { {
 for (; t < _break; t += dt) {
   state(_p, _ppvar, _thread, _nt);
  
}}
 t = _save;
 }}}

}

static void terminal(){}

static void _initlists(){
 double _x; double* _p = &_x;
 int _i; static int _first = 1;
  if (!_first) return;
 _slist1[0] = &(R1) - _p;  _dlist1[0] = &(DR1) - _p;
 _slist1[1] = &(R2) - _p;  _dlist1[1] = &(DR2) - _p;
 _slist1[2] = &(R1) - _p;  _dlist1[2] = &(DR1) - _p;
 _slist1[3] = &(R2) - _p;  _dlist1[3] = &(DR2) - _p;
 _slist1[4] = &(O1) - _p;  _dlist1[4] = &(DO1) - _p;
 _slist1[5] = &(O2) - _p;  _dlist1[5] = &(DO2) - _p;
 _slist1[6] = &(O1) - _p;  _dlist1[6] = &(DO1) - _p;
 _slist1[7] = &(O2) - _p;  _dlist1[7] = &(DO2) - _p;
 _slist1[8] = &(g) - _p;  _dlist1[8] = &(Dg) - _p;
_first = 0;
}

#if defined(__cplusplus)
} /* extern "C" */
#endif
