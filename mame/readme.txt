
                                M A M E

                    Multiple Arcade Machine Emulator

                  by Nicola Salmoria (MC6489@mclink.it)

please note that many people helped with this project, either directly or
by making source code available which I examined to write the drivers. I am
not trying to appropriate merit which isn't mine. See the acknowledgemnts
section for a list of contributors.


Here is a quick list of ther currently supported games; read on for details.
The list doesn't include variants of the same game.


                                         Accurate            Hi score
Game                         Playable?    colors?    Sound?    save?

Pac Man                        Yes         Yes        Yes       Yes
Ms Pac Man (bootleg)           Yes         Yes        Yes       Yes
Crush Roller                   Yes         Yes        Yes       Yes
Pengo                          Yes         Yes        Yes       Yes
Lady Bug                       Yes         Yes      No noise    Yes
Mr. Do!                        Yes         Yes        Yes       Yes
Mr. Do's Castle                 No          No         No       n/a
Crazy Climber                  Yes         Yes        Yes       Yes
Crazy Kong                     Yes         Yes        Yes       Yes
Donkey Kong                    Yes          No         No       Yes
Donkey Kong Jr.                Yes          No         No       Yes
Donkey Kong 3                  Yes          No         No       Yes
Mario Bros.                    Yes          No         No       Yes
Bagman                         Yes          No     Music only    No
Wizard of Wor                  Yes        Maybe        No        No
The Adventures of Robby Roto    No          No         No       n/a
Gorf                            No          No         No       n/a
Galaxian                       Yes         Yes      Limited     Yes
Pisces                         Yes         Yes      Limited      No
"Japanese Irem game"           Yes         Yes      Limited      No
War of the Bugs                Yes          No      Limited      No
Moon Cresta                    Yes         Yes      Limited     Yes
Moon Quasar                    Yes         Yes      Limited     Yes
Scramble                       Yes         Yes        Yes        No
Super Cobra                    Yes          No        Yes        No
The End                        Yes          No        Yes        No
Frogger                        Yes        Close       Yes        No
Amidar                         Yes        Close       Yes        No
Turtles                        Yes          No        Yes        No
Rally X                        Yes          No         No        No
Time Pilot                     Yes          No        Yes       Yes
Pooyan                         Yes          No        Yes       Yes
Phoenix                        Yes        Close     Limited      No
Pleiads                        Yes          No      Limited      No
Space Invaders                 Yes         Yes         No        No
Carnival                       Yes          No         No        No
Zaxxon                         Yes          No         No       Yes
Congo Bongo                    Yes          No         No       Yes
Bomb Jack                      Yes         Yes        Yes       Yes
Centipede                      Yes          No        Yes       Yes
Millipede                      Yes          No         No       Yes
Nibbler                        Yes          No         No       Yes
Moon Patrol                    Yes          No         No        No
Burger Time                    Yes         Yes        Yes       Yes
Lost Tomb                       No          No        Yes       n/a
Jump Bug                        No          No         No       n/a
Vanguard                       Yes          No         No        No



Acknowledgements
----------------

First of all, thanks to Allard van der Bas (avdbas@wi.leidenuniv.nl) for
starting the Arcade Emulation Programming Repository at
http://valhalla.ph.tn.tudelft.nl/emul8
Without the Repository, I would never have even tried to write an emulator.

If you find out something useful, submit it to avdbas@wi.leidenuniv.nl,
so it will be made available to everybody on the Repository page.

Z80Em Portable Zilog Z80 Emulator Copyright (C) Marcel de Kogel 1996,1997
   Note: the version used in MAME is slightly modified. You can find the
   original version at http://www.komkon.org/~dekogel/misc.html.
M6502 Emulator Copyright (C) Marat Fayzullin, Alex Krasivsky 1996
   Note: the version used in MAME is slightly modified. You can find the
   original version at http://freeflight.com/fms/.
Allegro library by Shawn Hargreaves, 1994/96
SEAL Synthetic Audio Library API Interface Copyright (C) 1995, 1996
   Carlos Hasan. All Rights Reserved.
Video modes created using Tweak 1.6b by Robert Schmidt, who also wrote
   TwkUser.c. Thanks to Chuck Cochems for the help in making them more
   compatible.
224x288 noscanlines and both 288x224 video modes provided by Valerio Verrando
  (v.verrando@mclink.it)
AY-3-8910 emulation by Ville Hallik (ville@physic.ut.ee) and Michael Cuddy
  (mcuddy@FensEnde.com).
POKEY emulator by Ron Fries (rfries@tcmail.frco.com).
UNIX port by Allard van der Bas (avdbas@wi.leidenuniv.nl) and Dick de Ridder
  (dick@ph.tn.tudelft.nl).

Phoenix driver provided by Brad Oliver (bradman@primenet.com), Mirko
   Buffoni (mix@lim.dsi.unimi.it) and Richard Davies (R.Davies@dcs.hull.ac.uk)
Mario Bros., Zaxxon, Bomb Jack, Burger Time and Donkey Kong 3 drivers provided
   by Mirko Buffoni (mix@lim.dsi.unimi.it)
Bomb Jack sound driver by Jarek Burczynski (pbk01@ikp.atm.com.pl).
Congo Bongo driver provided by Ville Laitinen (ville@sms.fi).
Millipede driver provided by Ivan Mackintosh (ivan@rcp.co.uk).
Donkey Kong sound emulation by Ron Fries (rfries@tcmail.frco.com).
Vanguard driver by Brad Oliver and Mirko Buffoni, based on code by Brian
   Levine.
Carnival driver completed by Mike Coates and Richard Davies.


Very special thanks to Sergio Munoz for the precious information about the
   Pengo sound hardware and colors.
Thanks to Paul Swan for the information on the Lady Bug sound hardware and
   Mr.Do! colors.
Big thanks to Gary Walton (garyw@excels-w.demon.co.uk) for too many things
   to mention them all.
Thanks to Simon Walls (wallss@ecid.cig.mot.com) for the color information
   on many games.
Information about the Crazy Climber machine hardware (including palette)
   and ROM encryption scheme provided by Lionel Theunissen
   (lionelth@ozemail.com.au).
Thanks to Andy Milne (andy@canetics.com) for the information on the Crazy
   Climber sound roms.
Crazy Kong emulation set up by Ville Laitinen (ville@sms.fi).
Special thanks to Brad Thomas (bradt@nol.net) and Gary Shepherdson for the
   extensive information on Donkey Kong and Donkey Kong Jr.
Info on Bagman, Galaxian, Moon Cresta and many other games taken from Arcade
   Emulator by Robert Anschuetz.
Pooyan information provided by Michael Cuddy and Allard van der Bas
Thanks to Mirko Buffoni for the Amidar and Frogger colors.
Thanks to Brad Thomas, Jakob Frendsen and Conny Melin for the info on Bomb
   Jack.
Thanks to Mike@Dissfulfils.co.uk for the information on the Moon Quasar
   encryption scheme.
Space Invaders information gathered from the Space Invaders Emulator by
   Michael Strutt (mstrutt@pixie.co.za)
Many thanks to Jim Hernandez for the information on Wizard of Wor hardware.
Thanks to Mike Coates (mike@dissfulfils.co.uk) for Carnival ROM placement
   indications and gfx info.
Colors for Donkey Kong, Donkey Kong Jr. and Mario Bros. derived from Kong
   emulator by Gary Shepherdson.
Colors for Amidar, Frogger and Zaxxon derived from SPARCADE by Dave Spicer.
Thanks to Brad Oliver, Marc Vergoossen (marc.vergoossen@pi.net) and Richard
   Davies (R.Davies@dcs.hull.ac.uk) for help with Donky Kong Jr. colors.
Thanks to Marc Vergoossen and Marc Lafontaine (marclaf@sympatico.ca) for
   Zaxxon colors.
Thanks to Marc Lafontaine for Congo Bongo colors.
Centipede information taken from Centipede emulator by Ivan Mackintosh, MageX
   0.3 by Edward Massey and memory map by Pete Rittwage.
Info on Burger Time taken from Replay 0.01a by Kevin Brisley (kevin@isgtec.com)
Thanks to Chris Hardy (chrish@kcbbs.gen.nz) for info on Moon Patrol.
Thanks to Dave W. (hbbuse08@csun.edu) for all his help.
Thanks to Doug Jefferys (djeffery@multipath.com) for Crazy Kong color
   information.
Thanks to Philip Chapman (Philip_Chapman@qsp.co.uk) for useful feedback on
   Bomb Jack.
Thanks to Mike Cuddy for Pooyan and Time pilot colors.
Thanks to Thomas Meyer for Moon Patrol screenshots.
Many thanks to Steve Scavone (krunch@intac.com) for his invaluable help with
   Wizard of Wor and related games.
-vesascan and -vesaskip implemented by Bernd Wiebelt
   (bernardo@studi.mathematik.hu-berlin.de)



Usage
-----

MAME [name of the game to run] [options]

for example

MAME mspacman -nosound   will run Ms Pac Man without sound

options:
-noscanlines  use alternate video mode (not availble in all games). Use this
              if the default mode doesn't work with your monitor/video card.
-vesa         use standard 640x480x256 VESA mode instead of custom video mode.
              Use this as a last resort if -noscanlines doesn't solve your
              video problems.
-vesascan     use a VESA 800x600 screen to simulate scanlines. This is much
              slower than the other video modes. Use this if you want
              scanlines and the default video mode doesn't work.
-vesaskip n   similar to -vesascan, but use a 640x480 screen instead of
              800x600. Since most games use a screen taller than 240 lines,
              it won't fit in the screen - n sets the initial number of lines
              to skip at the top of the screen. You can adjust the position
              while the game is running using the PGUP and PGDOWN keys.
-vgafreq n    where n can be 0 (default) 1, 2 or 3.
              use different frequencies for the custom video modes. This
              could reduce flicker, especially in the 224x288noscanlines
              mode. WARNING: THE FREQUENCIES USED MIGHT BE WAY OUTSIDE OF
              YOUR MONITOR RANGE, AND COULD EVEN DAMAGE IT. USE THESE OPTIONS
              AT YOUR OWN RISK.
-vsync        syncronize video display with the video beam instead of using
              the timer. This works best with -noscanlines and the -vesaxxx
              modes. Use F11 to check your actual frame rate - it should be
              around 60. If it is lower, try to increase it with -vgafreq (if
              you are using a tqeked video mode) or use your video board
              utilities to set the VESA refresh rate to 60 Hz.
              Note that when this option is turned on, speed will NOT
              downgrade nicely if your system is not fast enough.
-soundcard n  select sound card (if this is not specified, you will be asked
              interactively)
-nojoy        don't poll joystick
-log          create a log of illegal memory accesses in ERROR.LOG
-frameskip n  skip frames to speed up the emulation. For example, if the game
              normally runs at 60 fps, "-skipframe 1" will make it run at 30
              fps, and "-skipframe 2" at 20 fps. Use F11 to check the fps your
              computer is actually displaying. If the game is too slow,
              increase the frameskip value. Note that this setting can also
              affect audio quality (some games sound better, others sound
              worse).


The following keys work in all emulators:

3       Insert coin
1       Start 1 player game
2       Start 2 players game
Tab     Change dip switch settings
P       Pause
F3      Reset
F4      Show the game graphics. Use cursor keys to change set/color, F4 to exit.
F11     Toggle fps counter
F10     Toggle speed throttling
F12     Save a screen snapshot
ESC     Exit emulator



Pac Man ("pacman")
------------------

Arrows  Move around
F1      Skip level
F2      Test mode
CTRL    Speed up cheat

Clones supported:
  Pac Man modification ("pacmod")
  Namco Pac Man ("namcopac")
  Hangly Man ("hangly")
  Puck Man ("puckman")
  Piranha ("piranha")

Known issues:
- Blinky and Pinky seem to be shifted one pixel to the right. This is really
  annoying, but I can't seem to be able to understand why. Maybe there is an
  additional "sprite offset" register somewhere? Or did the original just
  behave this way?
  Note that we can't fix it by just moving sprites 0 and 1 one pixel to the
  left, because when Pac Man eats a power pill the sprites order is changed
  so that Pac Man is drawn over the ghosts. It becomes sprite 0, and Blinky
  becomes sprite 4.



Ms Pac Man ("mspacman")
-----------------------

Arrows  Move around
F1      Skip level
F2      Test mode
CTRL    Speed up cheat

Known issues:
- Blinky and Pinky seem to be shifted one pixel to the right. This is really
  annoying, but I can't seem to be able to understand why. Maybe there is an
  additional "sprite offset" register somewhere? Or did the original just
  behave this way?
  Note that we can't fix it by just moving sprites 0 and 1 one pixel to the
  left, because when Pac Man eats a power pill the sprites order is changed
  so that Pac Man is drawn over the ghosts. It becomes sprite 0, and Blinky
  becomes sprite 4.



Crush Roller ("crush")
----------------------

Crush Roller is a hacked version of Make Trax, modified to run on a
Pac Man board.

Arrows  Move around
F1      Skip level

Known issues:
- There's the same problem with sprites as in Pac Man, but here it could be
  fixed without apparent side effects.



Pengo ("pengo")
---------------

Arrows  Move around
CTRL    Push
F1      Skip level
F2      Test mode

Clones supported:
  Penta ("penta")



Lady Bug ("ladybug")
--------------------

Arrows  Move around
F1      Skip level

Known issues:
- The noise generator is not emulated yet.



Mr. Do! ("mrdo")
----------------

Arrows  Move around
CTRL    Fire
F1      Skip level
CTRL+F3 Test mode

Clones supported:
  Mr. Lo! ("mrlo")

Known issues:
- The noise generator is not emulated yet, but I think Mr. Do! doesn't
  use it anyway.



Mr. Do's Castle ("docastle")
----------------------------

Not working yet!



Crazy Climber ("cclimber")
--------------------------

E,S,D,F Left joystick
I,J,K,L Right joystick
F1      Skip level

Clones supported:
  Japanese version ("ccjap")
  bootleg version ("ccboot")



Crazy Kong ("ckong")
--------------------

This Donkey Kong clone runs on the same hardware as Crazy Climber, most
notable differencies being a larger character set and the display rotated
90 degrees.

Arrows  Move around
CTRL    Jump

Known issues:
- Some problems with sound



Donkey Kong ("dkong")
--------------------

Arrows  Move around
CTRL    Jump



Donkey Kong Jr. ("dkongjr")
---------------------------

Runs on hardware similar to Donkey Kong

Arrows  Move around
CTRL    Jump



Donkey Kong 3 ("dkong3")
------------------------

Runs on hardware similar to Donkey Kong

Arrows  Move around
CTRL    Fire
F1      Test (keep it pressed - very nice, try it!)



Mario Bros. ("mario")
---------------------

Runs on hardware similar to Donkey Kong

Arrows  Move around player 1
CTRL    Jump player 1
Z,X     Move around player 2
SPACE   Jump player 2
F1      Test (keep it pressed - very nice, try it!)



Bagman ("bagman")
-----------------

Arrows  Move around
CTRL    Action



Wizard of Wor ("wow")
---------------------

Arrows  Move around
CTRL    Fire
F2      Test mode (keep it pressed)
The original machine had a special joystick which could be moved either
partially or fully in a direction. Pushing it slightly would turn around the
player without moking it move. The emulator assumes that you are always
pushing the joystick fully, to simulate the "half press" you can press Alt.


Known issues:
- No background stars, no fade in/fade out.



The Adventures of Robby Roto ("robby")
--------------------------------------

This game runs on the same hardware as Wizard of Wor, but doesn't work yet. I
still haven to check the loading address of the ROMs.



Gorf ("gorf")
--------------------------------------

This game runs on the same hardware as Wizard of Wor, but doesn't work yet.
It boots, shows some text on the screen and that's all.



Galaxian ("galaxian")
---------------------

Original version with Namco copyright

Arrows  Move around
CTRL    Fire
F2      Test mode

  original with Midway copyright ("galmidw")
  and several bootlegs:
  one with Namco copyright ("galnamco")
  Super Galaxian ("superg")
  Galaxian Part X ("galapx")
  Galaxian Part 1 ("galap1")
  Galaxian Part 4 ("galap4")
  Galaxian Turbo ("galturbo")

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.



Pisces ("pisces")
-----------------

This runs on a modified Galaxian board.

Arrows  Move around
CTRL    Fire

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.
- What do the dip switches do?



"Japanese Irem game" ("japirem")
--------------------------------

This runs on a modified Galaxian board.

Arrows  Move around
CTRL    Fire

Clones supported:
  Uniwars ("uniwars")

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.
- What does dip switch 6 do?



War of the Bugs ("warofbug")
----------------------------

This runs on the same hardware as Galaxian.

Arrows  Move around
CTRL    Fire

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.
- What do the dip switches do?



Moon Cresta ("mooncrst")
------------------------

This runs on a hardware very similar to Galaxian.
The ROMs are encrypted. Nichibutsu copyright.

Arrows  Move around
CTRL    Fire

Clones supported:
  Unencrypted version ("mooncrsb")

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.
- What do the dip switches do?



Moon Quasar ("moonqsr")
-----------------------

This runs on a modified Moon Cresta board.

Arrows  Move around
CTRL    Fire

Known issues:
- Only one sound channel is emulated, and I'm not sure it's correct.
- The star background is probably not entirely accurate.



Scramble ("scramble")
---------------------

The video hardware is very similar to Galaxian, main differences being that
bullets are not vertical lines and the star background doesn't scroll.

Arrows  Move around
CTRL    Fire
ALT     Bomb

Clones supported:
  Battle of Atlantis ("atlantis") [I don't know what most of the dip switches
                                   do, and you get a massive 14 credits per
                                   coin - now that's what I call good value
                                   for money! ;-)]

Known issues:
- The star background is probably not entirely accurate. Also, maybe it should
  be clipped at the top and bottom of the screen?



Super Cobra ("scobra")
----------------------

Runs on the same hardware as Scramble.
This is the version with Stern copyright.

Arrows  Move around
CTRL    Fire
ALT     Bomb

Clones supported:
  Konami copyright ("scobrak")
  bootleg version ("scobrab")

Known issues:
- The star background is probably not entirely accurate. Also, maybe it should
  be clipped at the top and bottom of the screen?



The End ("theend")
------------------

This runs on a Scramble hardware.

Arrows  Move around
CTRL    Fire

Known issues:
- The star background is probably not entirely accurate. Also, maybe it should
  be clipped at the top and bottom of the screen?



Frogger ("frogger")
-------------------

Arrows  Move around

Clones supported:
  alternate version, smaller, with different help, but still (C) Sega 1981
     ("frogsega")
  bootleg version, which runs on a modified Scramble board ("froggers")



Amidar ("amidar")
-----------------

Arrows  Move around
CTRL    Jump

Clones supported:
  Japanese version ("amidarjp"). This version has a worse attract mode and
                                 does not display the number of jumps left.

Known issues:
- What do the dip switches do?



Turtles ("turtles")
-------------------

This runs on the same hardware as Amidar

Arrows  Move around
CTRL    Bomb

Known issues:
- What do the dip switches do? I'm obviously missing something, becasue the
  game plays in unlimited lives mode.



Rally X ("rallyx")
------------------

Arrows  Move around
CTRL    Smoke
F2      Test

Known issues:
- Sprites are not turned off appropriately.
- Cars are not displayed on the radar screen.
- I don't know if I reproduced the layout of the screen coreectly.



Time Pilot ("timeplt")
----------------------

Arrows  Move around
CTRL    Fire

Clones supported:
  bootleg version ("spaceplt")

Known issues:
- The ROM copyright message and the dipswitch menu are unreadable. Time Pilot
  doesn't have consecutive letters in the graphics ROms, I'll have to modify
  my text routines to handle that.
- This game uses double-width sprites for the clouds, but I haven't yet figured
  out they are selected. The code is currently a hack - just double the sprites
  which I know are used for clouds...
- The memory mapped read port at c000 puzzles me...


Pooyan ("pooyan")
-----------------

Runs on hardware similar to Time Pilot.

Arrows  Move around
CTRL    Fire

Known issues:
- The characters seem to use 16 color codes, however the color code for many
  characters has bit 4 set. I don't know what it's for.



Phoenix ("phoenix")
-------------------

Arrows  Move around
CTRL    Fire
ALT     Barrier



Pleiads ("pleiads")
---------------------

This runs on the same hardware as Phoenix.

Arrows  Move around
CTRL    Fire
ALT     Teleport



Space Invaders ("invaders")
---------------------------

Arrows  Move around
CTRL    Fire

Clones supported (some of these have wrong colors, and the dip switch menu
      doesn't work):
  Super Earth Invasion ("earthinv")
  Space Attack II ("spaceatt")
  Space Invaders Deluxe ("invdelux") (doesn't work yet)
  Galaxy Wars ("galxwars")
  Lunar Rescur ("lrescue")
  Destination Earth ("desterth")

Known issues:
- The color stripes are not placed correctly



Carnival ("carnival")
---------------------

Arrows  Move around
CTRL    Fire



Zaxxon ("zaxxon")
---------------------

Arrows  Move around
CTRL    Fire



Congo Bongo ("congo")
---------------------

Runs on the same hardware as Zaxxon.

Arrows  Move around
CTRL    Jump



Bomb Jack ("bombjack")
----------------------

Arrows  Move around
CTRL    Jump

Press fire to skip the ROM/RAM test at the beginning.

In the dip switch menu, DIFFICULTY 1 refers to the speed of the mechanical
bird, while DIFFICULTY 2 to the number and speed of enemies.
SPECIAL refers to how often the (E) and (S) coins appear.

Known issues:
- Colors are accurate, but not entirely: the original machine uses 12 bits
  (4 bits per pen), while I currently use only 8 bits (3 bits for red and
  green, 2 bits for blue).
- There is a bit in the sprite attributes which I don't know what means:
  it seems to be set only when the (B) materializes.
- The INITIAL HIGH SCORE setting doesn't only set that, it does something
  else as well - but I don't know what.



Centipede ("centiped")
----------------------

Arrows  Move around
CTRL    Fire

Known issues:
- What is the clock speed of the original machine? I'm currently using 1Mhz,
  I don't know if the game runs correctly.
- The game awards you 18 credits on startup



Millipede ("milliped")
----------------------

As you can imagine, this runs on the same hardware as Centipede.

Arrows  Move around
CTRL    Fire

Known issues:
- What is the clock speed of the original machine? I'm currently using 1Mhz,
  I don't know if the game runs correctly.
- High scores don't seem to work.



Nibbler ("nibbler")
-------------------

Arrows  Move around
F1      Skip level

Known issues:
- What is the clock speed of the original machine? I'm currently using 1Mhz
- Some input bits seem to be used as debug controls - quite interesting, but
  I haven't investigated yet.



Moon Patrol ("mpatrol")
-----------------------

Arrows  Move around
CTRL    Fire
ALT     Jump
F2+F3   Test mode (press and release, then be patient. After the RAM/ROM
                   tests, press 2 for an additional menu of options, then
                   left/right to choose the option, and 1 to select it)

Clones supported:
  bootleg version, called Moon Ranger ("mranger")

Known issues:
- No background graphics. I don't know where to place them... can anyone
  provide a screen snapshot?



Burger Time ("btime")
---------------------

Arrows  Move around
CTRL    Pepper
F1      \  Various tests.
F2      |  Use F1 to cycle through tests while in test mode.
F1+F2   /

Clones supported:
  different ROM set, without Midway copyright and different demo ("btimea")

Known issues:
- Sprites are not turned off appropriately



Lost Tomb ("losttomb")
----------------------

This runs on a Super Cobra hardware.

Known issues:
- Not playable. Crashes during demo. Graphics are garbled. I think the ROMs
  are corrupted.



Jump Bug ("jumpbug")
--------------------

This seems to run on hardware similar to Super Cobra.

Known issues:
- Not playable. Crashes during demo.



Vanguard ("vanguard")
--------------------

Runs on hardware similar to Nibbler.

Arrows  Move around
S,D,E,F Fire
