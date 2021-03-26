/*
 * wispr_config.h
 *
 * Created: 5/26/2020
 *  Author: Chris
 */ 


#ifndef WISPR_CONFIG_H_
#define WISPR_CONFIG_H_

extern void wispr_config_menu(wispr_config_t *config, int timeout);
extern void wispr_config_set_default(wispr_config_t *config);
extern void wispr_config_print(wispr_config_t *config);

#endif /* WISPR_CONFIG_H_ */