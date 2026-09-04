#include "adrc.h"
#include <math.h>

/* Both limits 0 means "no limits" - otherwise every default-initialised
 * cfg would be stuck at an output of exactly zero. */
static double clamp(const adrc_cfg *c, double u)
{
    if (c->u_min == 0.0 && c->u_max == 0.0) return u;
    if (u < c->u_min) return c->u_min;
    if (u > c->u_max) return c->u_max;
    return u;
}

int adrc_init(adrc *s, const adrc_cfg *cfg)
{
    int i;

    if (!s || !cfg) return ADRC_EARG;
    if (cfg->order != 1 && cfg->order != 2) return ADRC_EARG;
    if (!isfinite(cfg->b0) || cfg->b0 == 0.0) return ADRC_EARG;
    if (!isfinite(cfg->wc) || cfg->wc <= 0.0) return ADRC_EARG;
    if (!isfinite(cfg->wo) || cfg->wo <  0.0) return ADRC_EARG;
    if (!isfinite(cfg->u_min) || !isfinite(cfg->u_max)) return ADRC_EARG;
    if (cfg->u_min > cfg->u_max) return ADRC_EARG;

    s->cfg = *cfg;
    if (s->cfg.wo == 0.0) s->cfg.wo = 5.0 * s->cfg.wc;
    for (i = 0; i < 3; i++) s->z[i] = 0.0;
    s->u = 0.0;
    return ADRC_OK;
}

void adrc_reset(adrc *s, double measurement)
{
    if (!s) return;
    s->z[0] = isfinite(measurement) ? measurement : 0.0;
    s->z[1] = 0.0;
    s->z[2] = 0.0;
    s->u    = 0.0;
}

double adrc_update(adrc *s, double setpoint, double measurement, double dt)
{
    double wc, wo, b0, e, u0, u;

    if (!s) return 0.0;
    if (s->cfg.order != 1 && s->cfg.order != 2) return 0.0;   /* no adrc_init */
    if (!isfinite(setpoint) || !isfinite(measurement)) return s->u;
    if (!isfinite(dt) || dt <= 0.0) return s->u;

    wc = s->cfg.wc;
    wo = s->cfg.wo;
    b0 = s->cfg.b0;

    /* Control law, from the estimates the observer has right now. The last
     * term cancels the estimated disturbance, which is the whole trick:
     * whatever the observer blames for the plant not behaving - load,
     * friction, a b0 that is off - gets subtracted back out. */
    if (s->cfg.order == 1) {
        u0 = wc * (setpoint - s->z[0]);
        u  = (u0 - s->z[1]) / b0;
    } else {
        u0 = wc * wc * (setpoint - s->z[0]) - 2.0 * wc * s->z[1];
        u  = (u0 - s->z[2]) / b0;
    }
    u = clamp(&s->cfg, u);

    /* Propagate with the clamped u, not the wanted one: the observer sees
     * what the plant actually got, so a saturated output cannot wind the
     * disturbance estimate up. That is the anti-windup, and it is free. */
    e = measurement - s->z[0];
    /* ponytail: forward Euler; upgrade to exact discretisation if wo*dt > 0.2 */
    if (s->cfg.order == 1) {
        s->z[0] += dt * (s->z[1] + b0 * u + 2.0 * wo * e);
        s->z[1] += dt * (wo * wo * e);
    } else {
        s->z[0] += dt * (s->z[1] + 3.0 * wo * e);
        s->z[1] += dt * (s->z[2] + b0 * u + 3.0 * wo * wo * e);
        s->z[2] += dt * (wo * wo * wo * e);
    }

    s->u = u;
    return u;
}
