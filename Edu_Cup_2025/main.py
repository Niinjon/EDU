#!/usr/bin/env pybricks-micropython

#Good luck have fun 😊

from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile
from decimal import *
import math
from pybricks.iodevices import Ev3devSensor

class HitechnicSensor(Ev3devSensor):
    def __init__(self, port):
        super().__init__(port)
        self.path = '/sys/class/lego-sensor/sensor' +str(self.sensor_index)

    def get_color(self):
        return self.read('COLOR')[0]

class KDIErobot(DriveBase):
    def __init__(self, balmotor, jobbmotor, kerekatmero, tengelytav, gyroport, tengelyelore):
        super().__init__(balmotor, jobbmotor, kerekatmero, tengelytav)
        #self.balfeny = ColorSensor(Port.S3)
        #self.jobbfeny = ColorSensor(Port.S1)
        self.gyro = GyroSensor(gyroport)
        self.a = 400
        self.aktualszog = -90
        self.aktualx = 1570 #740 #kezdo - x: 745 y: 170
        self.aktualy = 50 #160
        
        self.fesziteirany = 1

        #self.heading_control.pid(10, 0, 0, 1, 1, 0)#200, 600, 2, 11, 2, 0
        #self.heading_control.limits(387, 1547, 100)

        self.distance_control.pid(120, 150, 4, 6, 2, 0) # default 80, 150, 10, 20, 4, 0 160, 150, 3, 10, 1, 0 240, 150, 3, 3, 1, 0
        self.distance_control.limits(500, 500, 100)#620, 700, 100
        self.heading_control.pid(25, 100, 2, 15, 3, 0) #30, 100 (Miki teszt)   #DEFAULT::200, 600, 2, 15, 3, 0
        self.maxseb = self.distance_control.limits()[0]
        #self.heading_control.limits(387, 1547, 100)
        self.timer = StopWatch()
        #self.timer1 = StopWatch()
        #self.file = open("teszt.txt", "w")
        #self.file.write("KEZDÉS")

    

        #self.lista = []

    def init_gyro(self, angle):
        self.aktualszog = angle
        self.gyro.reset_angle(angle)
        wait(1000)

    def gyroturn(self, angle):
        #self.file.write("!!!!!!!!!!GYROTURN!!!!!!!!!!" + "\n")
        #self.timer.reset()
        eredetiszog = angle
        irany = (angle / abs(angle)) if angle != 0 else 1

        ciklusatlag_tavolsag = 0.6 #0.26 #0.5 volt
        startangle = self.gyro.angle()
        angle = angle - (startangle - self.aktualszog)
        #celszog = (startangle + angle) - (irany * 3)
        steer = 110 #360
         
        erkezoseb = 42
        lassitoszog = 80

        if abs(angle) > lassitoszog:
            lassitasmertek = (erkezoseb -steer)/(lassitoszog/ciklusatlag_tavolsag)
        else:
            steer = ((steer - erkezoseb) / lassitoszog) * abs(angle) + erkezoseb
            lassitasmertek = (erkezoseb-steer)/((abs(angle) if angle != 0 else 1)/ciklusatlag_tavolsag)
        #self.file.write("kezdoaktualszog: " + str(self.aktualszog) + "\n")
        #self.file.write("angle: " + str(angle) + "\n")
        ##self.file.write("irány: " +str(irany) + "\n")
        #self.file.write("startangle: " + str(startangle) + "\n")
        ##self.file.write("lassításmérték: " + str(lassitasmertek) + "\n")
        hatralevo = abs(angle) - abs(self.gyro.angle() - startangle)
        ##self.file.write("hátralevő: " + str(hatralevo) + "\n")
        #steer = steeralap - (abs(gyro.angle() - startangle)/1.5)
        
        
        # -/+ 90: jól fordul nem mozdul /1 centi előre,jól fordul
        # -/+ 180: soha jobbat / félcenti előre,jól fordul
        # -/+ 270: jól fordul nem csúszik el / 1 centin belül előre jól fordul 


        if irany == 1:
            #if angle >90:
            #
            #elif :
            #
            #else:
            #
            # 
            eltolodas = 2 #3
            
            if angle > 180:
                eltolodas = 1 #4
        else:
            if angle < -180:
                eltolodas = 4 #1 5 6
            elif angle < -90:
                eltolodas = 2 #5 4 maradhat
            else:
                eltolodas = 1 #3 4 
        if abs(angle) <= 5:
            eltolodas = 4

        while hatralevo > eltolodas:  #3 1 2
            hatralevo = abs(angle) - abs(self.gyro.angle() - startangle)
            if steer > erkezoseb and hatralevo < lassitoszog + 4: 
                steer += lassitasmertek
            ##self.file.write("Steer: " + str(steer) + "\t")
            ##self.file.write("Sebesseg" + str(robot2.state()[3]) + "\n")
            self.drive(0, steer*irany)
        
        #ev3.screen.draw_text(0,40,robot2.state()[3])
        self.stop(Stop.BRAKE)
        self.stop(Stop.BRAKE)
        wait(400)



        eltolasharomszoga,eltolasharomszogb = self.eltolasharomszogszamitas(angle)

        #eltolasharomszogc = 0
        #if angle > 270:
        #    eltolasharomszogc = 10
        #elif angle < 270 and angle > 0:
        #    eltolasharomszogc = (angle * (10/270))
        #if self.aktualszog != 0:
        #    aktualszogirany = self.aktualszog / abs(self.aktualszog)
        #else:
        #    aktualszogirany = 1
        #eltolasharomszogszog = (abs(startangle + aktualszogirany * 0) % 360) * aktualszogirany #
        #eltolasharomszoga = math.sin(math.radians(eltolasharomszogszog)) * eltolasharomszogc*-1
        #eltolasharomszogb = math.cos(math.radians(eltolasharomszogszog)) * eltolasharomszogc
#
        #eltolasharomszoga = round(eltolasharomszoga)
        #eltolasharomszogb = round(eltolasharomszogb)

        self.aktualszog += eredetiszog

        self.aktualx += eltolasharomszogb
        self.aktualy += eltolasharomszoga
        ##self.file.write("eltolasharomszogc: " + str(eltolasharomszogc) + "\n")
        ##self.file.write("aktualszogirany: " + str(aktualszogirany) + "\n")
        ##self.file.write("eltolasharomszogszog: " + str(eltolasharomszogszog) + "\n")
        ##self.file.write("eltolasharomszoga: " + str(eltolasharomszoga) + "\n")
        ##self.file.write("eltolasharomszogb: " + str(eltolasharomszogb) + "\n")
        ##self.file.write("aktualx: " + str(self.aktualx) + "\n")
        ##self.file.write("aktualy: " + str(self.aktualy) + "\n")
        #self.file.write("gyro.angle: " + str(self.gyro.angle()) + "\n")
        #print("turn fordulando: " + str(angle)+"\n")
        #print("turn tenyleges szog: " + str(gyro.angle()))
        #self.file.write("veglegesaktszog: " + str(self.aktualszog)+"\n")
        #wait(400)

    def gyrokovetesceltavhoz(self, irany, celtav, celszog, sebesseg): 
        #print("gyrokov tav: " + str(celtav))
        #print("gyrokov szog: " + str(celszog))
        self.feszitesirany = irany
        kp = 1.3 #3 16 8 1.3
        ki = 0.002 #0.0002 0.0004
        kd = 6.2 #9 180 6 6.5
        error = 0
        lasterror = 0
        derivative = 0
        integral = 0
        lassitotav = 0
        lassitasido = 0
        mentettem = False
        lassitasmertek = 0
        megalloseb = 60
        reakcionyujtas = 1.3 #1.3
        seb = sebesseg #self.distance_control.limits()[0]
        aktualtav = self.state()[0]
        hatralevo = celtav
        self.drive(seb * irany, 0)
        while hatralevo > 1: #20 15 6
            
            #print(gyro.angle())
            hatralevo = celtav - abs(self.state()[0] - aktualtav)
            error = celszog - self.gyro.angle()
            integral += error
            derivative = error - lasterror
            lasterror = error
            steer = kp * error + ki * integral + kd * derivative
            if not mentettem:
                # d = vi*t - 1/2*a*t**2
                lassitotav = ((abs(self.state()[1]) ** 2) / (2 * self.a)) - ((megalloseb**2)/(2*self.a))
                lassitasido = ((abs(self.state()[1]) / self.a) - (megalloseb/self.a))*0.77 #0.55
            if lassitotav * reakcionyujtas >= hatralevo:
                if not mentettem:
                    #kp = 2
                    #kd = 4
                    maxseb = abs(self.state()[1])
                    seb = maxseb
                    mentettem = True
                if seb > megalloseb:
                    lassitasmertek = int((megalloseb-maxseb)/(lassitasido/(self.timer.time()/1000)))
                    seb += lassitasmertek
                else:
                    seb = megalloseb
            self.timer.reset()
            self.drive(seb * irany, steer)
            ##self.file.write(str(self.state()[1]) + '  ')

        #ev3.screen.draw_text(0,40,robot2.state()[1])
            #passedtime = self.timer.time()
            #wait(30 - self.timer.time())
            #self.lista.append(passedtime)
        self.stop(Stop.BRAKE)
        self.stop(Stop.BRAKE)
        ##self.file.write('\nlassitotav: ' + str(lassitotav))
        ##self.file.write('\nlassitasido: ' + str(lassitasido))
        ##self.file.write('\nlassitasmertek: ' + str(lassitasmertek) + '\n')
        wait(400)
    '''
    def ratekovetesceltavhoz(self, irany, celtav, celszog):
        #print("gyrokov tav: " + str(celtav))
        #print("gyrokov szog: " + str(celszog))
        kp = 1.5 #3 16 8 95
        ki = 6
        kd = 30#9 180 120
        error = 0
        lasterror = 0
        derivative = 0
        integral = 0
        lassitotav = 0
        mentettem = False
        lassitasmertek = 0
        megalloseb = 80
        reakcionyujtas = 1.3
        seb = self.distance_control.limits()[0]
        aktualtav = self.state()[0]
        hatralevo = celtav
        self.drive(seb * irany, 0)
        szogret=celszog
        while hatralevo > 5:
            #print(gyro.angle())
            (self.timer1.time()/1000)
            szogret+=(self.gyro.speed()*(self.timer1.time()/1000))
            #self.file.write(str(szogret) + ' ')
            hatralevo = celtav - abs(self.state()[0] - aktualtav)
            error = celszog - szogret
            integral += error
            derivative = error - lasterror
            lasterror = error
            steer = kp * error + ki * integral + kd * derivative
            if not mentettem:
                # d = vi*t - 1/2*a*t**2
                lassitotav = ((abs(self.state()[1]) ** 2) / (2 * self.a)) - ((megalloseb**2)/(2*self.a))
                lassitasido = (abs(self.state()[1]) / self.a) - (megalloseb/self.a)
            if lassitotav * reakcionyujtas >= hatralevo:
                if not mentettem:
                    #kp = 2
                    #kd = 4
                    maxseb = abs(self.state()[1])
                    seb = maxseb
                    mentettem = True
                if seb > megalloseb:
                    lassitasmertek = int((megalloseb-maxseb)/(lassitasido/(self.timer.time()/1000)))
                    seb += lassitasmertek
                else:
                    seb = megalloseb
            self.timer.reset()
            self.timer1.reset()
            self.drive(seb * irany, steer)
            ##self.file.write(str(self.state()[1]) + '  ')

        #ev3.screen.draw_text(0,40,robot2.state()[1])
            #passedtime = self.timer.time()
            #wait(30 - self.timer.time())
            #self.lista.append(passedtime)
        self.stop(Stop.BRAKE)
        self.stop(Stop.BRAKE)
        #self.file.write('\nlassitotav: ' + str(lassitotav))
        #self.file.write('\nlassitasido: ' + str(lassitasido))
        #self.file.write('\nlassitasmertek: ' + str(lassitasmertek) + '\n')
        wait(300)
    '''

    def rgb_to_hsl(self, red, green, blue):
        r = red / 255.0
        g = green / 255.0
        b = blue / 255.0

        max_color = max(r,g,b)
        min_color = min(r,g,b)
        delta = max_color - min_color

        l = (max_color + min_color) / 2.0

        if delta < 0.00001:
            h = 0.0
            s = 0.0
        else:
            if l < 0.5:
                s = delta / (max_color + min_color)
            else:
                s = delta / (2.0 - max_color-min_color)

            delta_r = (((max_color-r)/6.0)+(delta / 2.0)) / delta
            delta_g = (((max_color-g)/6.0)+(delta / 2.0)) / delta
            delta_b = (((max_color-b)/6.0)+(delta / 2.0)) / delta

            if r == max_color:
                h = delta_b-delta_g
            elif g == max_color:
                h = (1.0 / 3.0) + delta_r - delta_b
            elif b == max_color:
                h = (2.0 / 3.0) + delta_g - delta_r
            if h < 0.0:
                h += 1.0
            if h > 1.0:
                h -= 1.0

        return h,s,l


    def eltolasharomszogszamitas(self, szog):
        eltolasharomszogc = 0
        eltolasharomszoga = 0
        eltolasharomszogb = 0
        eltolasirany = 0
        eltolasszog = 0
        kr_aktualszog = self.aktualszog * -1
        kr_szog = szog * -1
        

        
        #90 fok - 5 mm hátra 2? mm jobbra| 4 mm hátra
        #180 fok - 5 mm hátra | 4 mm hátra
        #270 fok - 2 mm hátra 2 mm balra 
        #if szog < 0 and szog >= -45:
        #    eltolasharomszogc = (5/45)*abs(kr_szog)
        #    eltolasirany = 180 #270
#
        #elif szog < -45 and szog >= -90:
        #    #eltolasharomszogc = (5/45)*abs(kr_szog) 
        #    eltolasharomszogb = (-3/45)*(abs(kr_szog)-45)
        #    eltolasharomszoga = (-1.66*eltolasharomszogb)-5
        #    eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #    eltolasirany =  math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90
#
        #elif szog < -90 and szog >= -180:
        #    eltolasharomszogb = (-0.5/180)*(abs(kr_szog)-90)
        #    eltolasharomszoga = (8*eltolasharomszogb)+28
        #   eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #   eltolasirany = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90 #180

        #lif szog < -180 and szog >= -360:
        #   eltolasharomszoga = 3-7.5*math.sin(math.radians(szog*-1/3+217)) #szog/6+187 
        #   eltolasharomszogb = -4-7.5*math.cos(math.radians(szog*-1/3+217))
        #   eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #   eltolasirany = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90 #225

        #if self.feszitesirany == 1:

        #else:


        #if szog > 90 and szog <= 180:
        #    eltolasirany = 180
        #    eltolasharomszogc = 5/90*(szog-90)





        #f szog < 0 and szog >= -45:
        #   eltolasharomszogc = (5/45)*abs(kr_szog)
        #   eltolasirany = 180 #270

        #lif szog < -45 and szog >= -90:
        #   #eltolasharomszogc = (5/45)*abs(kr_szog) 
        #   eltolasharomszogb = (-3/45)*(abs(kr_szog)-45)
        #   eltolasharomszoga = (-1.66*eltolasharomszogb)-5
        #   eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #   eltolasirany =  math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90

        #lif szog < -90 and szog >= -180:
        #   eltolasharomszogb = (-0.5/180)*(abs(kr_szog)-90)
        #   eltolasharomszoga = (8*eltolasharomszogb)+28
        #   eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #   eltolasirany = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90 #180

        #lif szog < -180 and szog >= -360:
        #   eltolasharomszoga = 3-7.5*math.sin(math.radians(szog*-1/3+217)) #szog/6+187 
        #   eltolasharomszogb = -4-7.5*math.cos(math.radians(szog*-1/3+217))
        #   eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
        #   eltolasirany = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb))) - 90 #225




        if szog > 0 and szog <= 90:
            eltolasharomszogc = (7/90)*abs(kr_szog)
            eltolasirany = 180 #270

        elif szog > 90 and szog <= 180:
            eltolasharomszoga = -7
            eltolasharomszogb = (-2/90)*(szog-90)
            #270
            eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
            eltolasszog = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb)))
            eltolasirany = 180+(eltolasszog+90)
        
        elif szog > 180 and szog <= 270: 
            eltolasharomszogb = (-1/90)*(szog-180)+-2
            eltolasharomszoga = -3*eltolasharomszogb-13
            eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
            eltolasszog = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb)))
            eltolasirany = 180+(eltolasszog+90)

        elif szog > 270 and szog <= 360:
            eltolasharomszogb = (3/90)*(szog-270)-3
            eltolasharomszoga = eltolasharomszogb/3-3
            eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
            eltolasszog = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb)))
            eltolasirany = 180+(eltolasszog+90)


        elif szog < 0 and szog >= -105:
            eltolasharomszoga = -2
            eltolasharomszogb = (-5/-105)*szog
            eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
            eltolasszog = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb)))
            eltolasirany = 180+(eltolasszog+90)

        elif szog < -105 and szog >= -270:
            eltolasharomszogb = 0+5*math.cos(math.radians(90/165*(abs(szog)-105)+180)) #(5/-165)*(szog+270)
            eltolasharomszoga = -2+5*math.sin(math.radians(90/165*(abs(szog)-105)+180))
            eltolasharomszogc = math.sqrt(eltolasharomszoga**2 + eltolasharomszogb**2)
            eltolasszog = math.degrees(math.atan2(round(eltolasharomszoga), round(eltolasharomszogb)))
            eltolasirany = 180+(eltolasszog+90)
        
        elif szog < -270 and szog >= -360:
            eltolasharomszogc = (5/90)*(szog+360)+2
            eltolassirany = 180

        
            


        
        #if abs(szog) > 270:
        #    eltolasharomszogc = 10
        #elif szog < 270 and szog> 0:
        #    eltolasharomszogc = (abs(szog) * (10 / 270))
        if kr_aktualszog+eltolasirany != 0:
            aktualszogirany = (kr_aktualszog+eltolasirany) / abs(kr_aktualszog+eltolasirany)
        else:
            aktualszogirany = 1
        eltolasharomszogszog = (abs(kr_aktualszog + eltolasirany) % 360) * aktualszogirany
        eltolasharomszoga = math.sin(math.radians(eltolasharomszogszog)) * eltolasharomszogc
        eltolasharomszogb = math.cos(math.radians(eltolasharomszogszog)) * eltolasharomszogc
#
        eltolasharomszoga = round(eltolasharomszoga)
        eltolasharomszogb = round(eltolasharomszogb)
        return eltolasharomszoga,eltolasharomszogb

    def haromszogszamitas(self, celx, cely, fix='x'): 
        #self.file.write("!!!!!!!!!!HAROMSZOGSZAMITAS!!!!!!!!!!" + "\n")
        #self.file.write("aktualszog hsz elott: " + str(self.aktualszog) + "\n")
        #self.file.write("celx: " + str(celx) + "\n")
        #self.file.write("cely: " + str(cely) + "\n")
        #self.file.write("aktualx: " + str(self.aktualx) + "\n")
        #self.file.write("aktualy: " + str(self.aktualy) + "\n")
        
        haromszoga = 0
        haromszogb = 0
        haromszoga = cely - self.aktualy
        haromszogb = celx - self.aktualx
        #self.file.write("haromszoga: " + str(haromszoga) + "\n")
        #self.file.write("haromszogb: " + str(haromszogb) + "\n")

        #print("haromszoga: " + str(haromszoga))
        #print("haromszogb: " + str(haromszogb))
        haromszogc = (haromszoga**2 + haromszogb**2)**0.5
        #print("haromszog tav: " + str(haromszogc))
        szog = math.atan2(round(haromszoga), round(haromszogb))#if haromszogb != 0 else 1)
        szog = round(math.degrees(szog))
        #print("haromszog szog: " + str(szog))
        #print('hc1 ' + str(haromszogc))
        haromszogc = round(haromszogc)
        #self.file.write("haromszogc: " + str(haromszogc) + "\n")
        #self.file.write("szog: " + str(szog) + "\n")
        ##self.file.write("yirany: " + str(yirany) + "\n")
        #print("haromszog akt: " + str(aktualszog))
        #print(szog)

        if szog*-1 != self.aktualszog:
            szogxirany = (szog*-1 - self.aktualszog) / abs(szog*-1 - self.aktualszog)
        else:
            szogxirany = 1
        szogelfordulas = ((abs(szog*-1 - self.aktualszog)) % 360) * szogxirany
        #self.file.write("szogxirany: " + str(szogxirany) + "\n")
        #print('2 ke ' + str(szogelfordulas))
        #szogelfordulas =  #((szogyirany - 1) * (-180)) + szogelfordulas
        szogelfordulas = round(szogelfordulas)
        #self.file.write("round szogelfordulas: " + str(szogelfordulas) + "\n")
        

        eltolasharomszoga,eltolasharomszogb = 0, 0#elf.eltolasharomszogszamitas(szogelfordulas)

        #eltolasharomszogc = 0
        #eltolasharomszoga = 0
        #eltolasharomszogb = 0
        #if abs(szogelfordulas) > 270:
        #    eltolasharomszogc = 10
        #elif szogelfordulas < 270 and szogelfordulas > 0:
        #    eltolasharomszogc = (abs(szogelfordulas) * (10 / 270))
        #if self.aktualszog != 0:
        #    aktualszogirany = self.aktualszog / abs(self.aktualszog)
        #else:
        #    aktualszogirany = 1
        #eltolasharomszogszog = (abs(self.aktualszog + aktualszogirany * 0) % 360) * aktualszogirany
        #eltolasharomszoga = math.sin(math.radians(eltolasharomszogszog)) * eltolasharomszogc*-1
        #eltolasharomszogb = math.cos(math.radians(eltolasharomszogszog)) * eltolasharomszogc

        #eltolasharomszoga = round(eltolasharomszoga)
        #eltolasharomszogb = round(eltolasharomszogb)
        
        
        aktxhelp = self.aktualx
        aktyhelp = self.aktualy

        aktxhelp += eltolasharomszogb
        aktyhelp += eltolasharomszoga
        
        
        ##self.file.write("eltolasharomszogc: " + str(eltolasharomszogc) + "\n")
        ##self.file.write("aktualszogirany: " + str(aktualszogirany) + "\n")
        ##self.file.write("eltolasharomszogszog: " + str(eltolasharomszogszog) + "\n")
        ##self.file.write("eltolasharomszoga: " + str(eltolasharomszoga) + "\n")
        ##self.file.write("eltolasharomszogb: " + str(eltolasharomszogb) + "\n")
        
        
        ##self.file.write("aktxhelp: " + str(aktxhelp) + "\n")
        ##self.file.write("aktyhelp: " + str(aktyhelp) + "\n")
        
        #print('aktxhelp ' + str(aktxhelp))
        #print('aktyhelp ' + str(aktyhelp))
        
        haromszoga = cely - aktyhelp
        haromszogb = celx - aktxhelp
        
        
        #self.file.write("haromszoga: " + str(haromszoga) + "\n")
        #self.file.write("haromszogb: " + str(haromszogb) + "\n")

        #print("haromszoga: " + str(haromszoga))
        #print("haromszogb: " + str(haromszogb))
        haromszogc = (haromszoga**2 + haromszogb**2)**0.5
        #print("haromszog tav: " + str(haromszogc))
        szog = math.atan2(round(haromszoga), round(haromszogb))#if haromszogb != 0 else 1)
        szog = round(math.degrees(szog))
        #print("haromszog szog: " + str(szog))
        #print('hc2 ' + str(haromszogc))
        haromszogc = round(haromszogc)
        #self.file.write("haromszogc: " + str(haromszogc) + "\n")
        #self.file.write("szog: " + str(szog) + "\n")
        ##self.file.write("yirany: " + str(yirany) + "\n")
        #if abs(szog) > 90:
        #    eltolasharomszogc = 10
        #else:
        #    eltolasharomszogc = (abs(szog) / 9)
        #if self.aktualszog != 0:
        #    aktualszogirany = self.aktualszog / abs(self.aktualszog)
        #else:
        #    aktualszogirany = 1
        #eltolasharomszogszog = (abs(self.aktualszog + aktualszogirany * 180) % 360) * aktualszogirany
        #self.eltolasharomszoga2 = math.sin(math.radians(eltolasharomszogszog)) * eltolasharomszogc
        #self.eltolasharomszogb2 = math.cos(math.radians(eltolasharomszogszog)) * eltolasharomszogc
        
        
        #szog = abs(szog) * xirany
        if szog*-1 != self.aktualszog:
            szogxirany = (szog*-1 - self.aktualszog) / abs(szog*-1 - self.aktualszog)
        else:
            szogxirany = 1
        szogelfordulas = ((abs(szog*-1 - self.aktualszog)) % 360) * szogxirany
        ##self.file.write("szogxirany: " + str(szogxirany) + "\n")
        #print('2 ke ' + str(szogelfordulas))
        #szogelfordulas =  #((szogyirany - 1) * (-180)) + szogelfordulas
        szogelfordulas = round(szogelfordulas)
        ##self.file.write("round szogelfordulas: " + str(szogelfordulas) + "\n")
        #print("haromszog elfordulas: " + str(szogelfordulas))
        #akttav = self.state()[0]
        #self.eltolasharomszoga = 0
        #self.eltolasharomszogb = 0
        #print(szogelfordulas)
        #print('a2 ' + str(self.eltolasharomszoga2))
        #print('b2 ' + str(self.eltolasharomszogb2))
        #print(szog)
        
        
        if fix == 'x':
            if haromszogb != 0:
                haromszogc=haromszogb / (math.cos(math.radians(szog) if szog != 90 else 1))
            haromszogc=round(haromszogc)
            haromszoga=math.sin(math.radians(szog))*haromszogc
            haromszoga=round(haromszoga)
        else:
            if haromszoga != 0:
                haromszogc=haromszoga / (math.sin(math.radians(szog) if szog != 0 else 1))
            haromszogc=round(haromszogc)
            haromszogb=math.cos(math.radians(szog))*haromszogc
            haromszogb=round(haromszogb)
        
        
        #self.file.write("új a: " + str(haromszoga) + "\n")
        #self.file.write("új b: " + str(haromszogb) + "\n")
        #self.file.write("új c: " + str(haromszogc) + "\n")
        #csak ha a szogelfordulas nem 0
        #motormozgás
        

        if szogelfordulas != 0:        
            self.gyroturn(szogelfordulas)
            

        self.gyrokovetesceltavhoz(1, haromszogc, self.aktualszog, self.maxseb)

        #print("eltolasharomszoga: " + str(round(eltolasharomszoga)))
        #print("eltolasharomszogb: " + str(round(eltolasharomszogb)))
        #print("eltolasharomszogszog: " + str(eltolasharomszogszog))

        #tenylegesmegtett = robot.distance() - akttav
        ##tenylegesmegtett = 807
        #tenylegesa = math.sin(math.radians(szog*xirany)) * tenylegesmegtett
        #tenylegesb = math.cos(math.radians(szog*xirany)) * tenylegesmegtett
        #tenylegesb = abs(tenylegesb) * yirany
        #tenylegesa = abs(tenylegesa) * xirany
        #print("tenylegesa :" + str(tenylegesa))
        #print("tenylegesb :" + str(tenylegesb))
        self.aktualx += haromszogb #+ self.eltolasharomszoga #aktualx + round(tenylegesa)
        self.aktualy += haromszoga #+ self.eltolasharomszogb #aktualy + round(tenylegesb)

        #self.file.write("aktualx hsz után: " + str(self.aktualx) + "\n")
        #self.file.write("aktualy hsz után: " + str(self.aktualy) + "\n")

        #self.file.write("aktualszog hsz utan: " + str(self.aktualszog) + "\n")

    def elore(self, tavolsag):
        #self.file.write("!!!!!!!!TOLATAS!!!!!!!!\n")
        wait(300)
        kovetendo = self.gyro.angle()
        kr_kovetendo = kovetendo * -1
        self.gyrokovetesceltavhoz(1, tavolsag, kovetendo, 150)
        if (kr_kovetendo) != 0:
            aktualszogirany = (kr_kovetendo) / abs(kr_kovetendo)
        else:
            aktualszogirany = 1
        
        eltolasharomszogszog = (abs(kr_kovetendo) % 360) * aktualszogirany
        eltolasharomszoga = math.sin(math.radians(eltolasharomszogszog)) * tavolsag
        eltolasharomszoga = round(eltolasharomszoga)
        eltolasharomszogb = math.cos(math.radians(eltolasharomszogszog)) * tavolsag
        eltolasharomszogb = round(eltolasharomszogb)
        #self.aktualszog = kovetendo

        ##self.file.write("kovetendo: " + str(kovetendo) + "\n" )
        ##self.file.write("gyroszög: " + str(self.gyro.angle()) + "\n" )
        ##self.file.write("aktualszog: " + str(self.aktualszog) + "\n")
        ##self.file.write("aktualszogirany: " + str(aktualszogirany) + "\n")
        ##self.file.write("eltolasharomszogszog: " + str(eltolasharomszogszog) + "\n")
        ##self.file.write("eltolasharomszoga: " + str(eltolasharomszoga) + "\n")
        ##self.file.write("eltolasharomszogb: " + str(eltolasharomszogb) + "\n")
        
        self.aktualx += eltolasharomszogb
        self.aktualy += eltolasharomszoga
        #self.file.write("aktualx: " + str(self.aktualx) + "\n")
        #self.file.write("aktualy: " + str(self.aktualy) + "\n")

    def tolatas(self, tavolsag):
        #self.file.write("!!!!!!!!TOLATAS!!!!!!!!\n")
        wait(300)
        kovetendo = self.gyro.angle()
        kr_kovetendo = kovetendo * -1
        self.gyrokovetesceltavhoz(-1, tavolsag, kovetendo, 150)
        if (kr_kovetendo+180) != 0:
            aktualszogirany = (kr_kovetendo+180) / abs(kr_kovetendo+180)
        else:
            aktualszogirany = 1
        
        eltolasharomszogszog = (abs(kr_kovetendo + 180) % 360) * aktualszogirany
        eltolasharomszoga = math.sin(math.radians(eltolasharomszogszog)) * tavolsag
        eltolasharomszoga = round(eltolasharomszoga)
        eltolasharomszogb = math.cos(math.radians(eltolasharomszogszog)) * tavolsag
        eltolasharomszogb = round(eltolasharomszogb)
        #self.aktualszog = kovetendo

        ##self.file.write("kovetendo: " + str(kovetendo) + "\n" )
        ##self.file.write("gyroszög: " + str(self.gyro.angle()) + "\n" )
        ##self.file.write("aktualszog: " + str(self.aktualszog) + "\n")
        ##self.file.write("aktualszogirany: " + str(aktualszogirany) + "\n")
        ##self.file.write("eltolasharomszogszog: " + str(eltolasharomszogszog) + "\n")
        ##self.file.write("eltolasharomszoga: " + str(eltolasharomszoga) + "\n")
        ##self.file.write("eltolasharomszogb: " + str(eltolasharomszogb) + "\n")
        
        self.aktualx += eltolasharomszogb
        self.aktualy += eltolasharomszoga
        #self.file.write("aktualx: " + str(self.aktualx) + "\n")
        #self.file.write("aktualy: " + str(self.aktualy) + "\n")
        
    def tengelyelore():
        gyrokovetesceltavhoz(1, 50, aktualszog)

    def szogrefordulas(self, rafordulando):
        aktszog = self.aktualszog
        irany = (aktszog / abs(aktszog)) if aktszog != 0 else 1
        aktszog = abs(aktszog)
        aktszog = (rafordulando - (aktszog % 360) * irany)
        #ev3.scree.draw_text(0, 40, aktszog)
        self.gyroturn(aktszog)

def pozit(szog):
    
    if robot2.gyro.angle() == 0:
        irany = 1
    else:
        irany = robot2.gyro.angle() / abs(robot2.gyro.angle())
    jobbfeny = jobbfenysensor.reflection()
    balfeny = balfenysensor.reflection()
    posittimer = StopWatch()
    posittimer.reset()

    while not ((abs(robot2.gyro.angle()) % 360) * irany == szog and (jobbfeny > 30 and jobbfeny < 42) and (balfeny > 45 and balfeny < 57)):# j 75,10 b 88,14
        jobbfeny = jobbfenysensor.reflection()
        balfeny = balfenysensor.reflection()

        if posittimer.time()>2000:
            break
        #j 9, 73 b 13, 89
        if jobbfeny > 47:
            jobbmotor.run(40)
        elif jobbfeny < 35:
            jobbmotor.run(-40)
        else:
            jobbmotor.stop(Stop.BRAKE)
        if balfeny > 57:
            balmotor.run(40)
        elif balfeny < 45:
            balmotor.run(-40)
        else:
            balmotor.stop(Stop.BRAKE)

def vonalratolatas(kezdo, irany):
    robot2.drive(100*irany, 0)
    if kezdo == 'B':
        while not jobbfenysensor.reflection()>60:
            pass
    while not jobbfenysensor.reflection()<35:
        pass
    while not jobbfenysensor.reflection()>60:
        pass

def hslszin():
    red, green, blue = fenysensor.rgb()

    hue, saturation, value = robot2.rgb_to_hsl(red,green,blue)

    if hue <0.1:
        return Color.RED
    elif hue<0.2:
        return Color.YELLOW
    elif hue<0.4:
        return Color.GREEN
    else:
        return Color.BLUE

def gombravar():
    while len(ev3.buttons.pressed()) == 0:
        pass
    wait(1000)

    # This program requires LEGO EV3 MicroPython v2.0 or higher.
    # Click "Open user guide" on the EV3 extension tab for more information.

    # Create your objects here.
ev3 = EV3Brick()

gat = Motor(Port.A)
kar = Motor(Port.D)
#robot = KDIErobot(balmotor, jobbmotor, kerekatmero, tengelytav, Port.S3)
balmotor = Motor(Port.B)
jobbmotor = Motor(Port.C)
#jobbmotor.control.limits(2000, 4000)
#balmotor.control.limits(2000, 4000)
#fenysensor = ColorSensor(Port.S1)
#balfenysensor = ColorSensor(Port.S3)
emelo = Motor(Port.A)
kar = Motor(Port.D)
#htcolor = HitechnicSensor(Port.S2)
kerekatmero = 62.4 #31.5 lánctalp #62.4 #kisfekete #87 nagykék  100.6 cross gumi 
tengelytav = 160
tengelyelore = 50
ev3.speaker.set_speech_options(language='en', voice='m1', speed=100, pitch=99)
ev3.speaker.set_volume(100,'_all_')
# Program
#robot.drive(1000)
robot2 = KDIErobot(balmotor, jobbmotor, kerekatmero, tengelytav, Port.S3, tengelyelore)

robot2.init_gyro(-90)
wait(3000)

'''
while len(ev3.buttons.pressed()) == 0:
    pass
wait(1000)
'''

#gombravar()

'''
#p1
kar.run_angle(100, 100)
robot2.haromszogszamitas(720,490)
kar.run_until_stalled(-200, Stop.HOLD)
robot2.haromszogszamitas(2125,750,'y')
robot2.tolatas(200)
robot2.elore(50)
robot2.haromszogszamitas(1740,890)
emelo.run_until_stalled(-300,Stop.HOLD)
robot2.tolatas(150)
robot2.elore(50)
robot2.haromszogszamitas(2040,960)
emelo.run_until_stalled(300,Stop.HOLD)
kar.run_angle(100, 120)
robot2.tolatas(400)
robot2.elore(50)
robot2.haromszogszamitas(1590,920)
'''

#kar.run_angle(-100, 220)
#kar.run_until_stalled(100,Stop.HOLD)
#wait(1000)
#kar.run_until_stalled(-100,Stop.HOLD)
#wait(500)

#robot2.gyroturn(-270)
#robot2.gyrokovetesceltavhoz(1,1500,robot2.aktualszog,600)

# ===== indulás =====
robot2.haromszogszamitas(robot2.aktualx,robot2.aktualy+70)

'''
# ===== Kocka 1 =====
robot2.haromszogszamitas(920,930) #1280 580
robot2.haromszogszamitas(260,480)#480 730
robot2.szogrefordulas(0)
robot2.tolatas(135)
robot2.szogrefordulas(-90)
robot2.elore(100)
robot2.gyroturn(17)
robot2.tolatas(30)
robot2.gyroturn(-7)
robot2.elore(170)
robot2.gyroturn(3)


# ===== Kocka 2 =====

robot2.elore(105)#130
robot2.gyroturn(17)
robot2.tolatas(40)
robot2.gyroturn(-7)
robot2.elore(100)
'''

'''

                    !!!!!!!!!!!!FONTOS!!!!!!!!!!!!!!!
    ===== Fixáld be amit kell alapértelmezés szerint x a fix =====


'''

# ===== 1. Kocka =====
robot2.haromszogszamitas(920,930) #1280 580
robot2.haromszogszamitas(265,480)#480 730
robot2.szogrefordulas(0)
robot2.tolatas(135)
robot2.szogrefordulas(-90)
robot2.elore(200)
robot2.haromszogszamitas(920,930)
robot2.haromszogszamitas(1200,440)
robot2.gyroturn(-150)
robot2.gyroturn(5)
robot2.tolatas(150)
robot2.haromszogszamitas(1680,300)
robot2.haromszogszamitas(robot2.aktualx,690,'y')
robot2.haromszogszamitas(2120,645,'y')
robot2.haromszogszamitas(1410,700)
robot2.haromszogszamitas(1130,940)


'''
# ===== Emberek =====
robot2.haromszogszamitas(2160,560)
robot2.haromszogszamitas(1730,740)
robot2.szogrefordulas(-90)
robot2.elore(50)
robot2.haromszogszamitas(1680,210)
robot2.haromszogszamitas(1180,510)
robot2.tolatas(100)
'''
#Well done 👍