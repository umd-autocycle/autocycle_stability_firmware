//
// Created by Misha on 12/10/2020.
//

#ifndef AUTOCYCLE_STABILITY_FIRMWARE_CONTROLLER_H
#define AUTOCYCLE_STABILITY_FIRMWARE_CONTROLLER_H

class Controller {
    public:
        virtual float control(float phi, float del, float dphi, float ddel, float phi_r, float del_r, float v, float dt) = 0;

};

#endif //AUTOCYCLE_STABILITY_FIRMWARE_CONTROLLER_H
