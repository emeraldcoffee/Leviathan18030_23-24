package org.firstinspires.ftc.teamcode.drive;

import com.acmerobotics.roadrunner.localization.Localizer;

interface CustomLocalizerImp {
    void updateBackdrop();
    void compensatedUpdateBackdrop();

    void smartUpdateBackdrop();
    void smartCompensatedUpdateBackdrop();
}
