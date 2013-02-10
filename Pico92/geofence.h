
/*
 AVA/ATLAS 70cm/2Mtr RTTY/APRS Tracker
 
 By Anthony Stirk M0UPU / James Coxon M6JCX

 Additional code from :
 Project Swift
 Steve Randall

 Copyright 2010-2012 Nigel Smart <nigel@projectswift.co.uk>            
 And Philip Heron <phil@sanslogic.co.uk>                               
 Additional Help from Jon Sowman & Adam Greig of Cambridge University  
 Spaceflight (www.cusf.co.uk)                                          

/* This program is free software: you can redistribute it and/or modify  */
/* it under the terms of the GNU General Public License as published by  */
/* the Free Software Foundation, either version 3 of the License, or     */
/* (at your option) any later version.                                   */
/*                                                                       */
/* This program is distributed in the hope that it will be useful,       */
/* but WITHOUT ANY WARRANTY; without even the implied warranty of        */
/* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the          */
/* GNU General Public License for more details.                          */
/*                                                                       */
/* You should have received a copy of the GNU General Public License     */
/* along with this program. If not, see <http://www.gnu.org/licenses/>.  */

#include <stdint.h>
#include <avr/pgmspace.h>

/* 
  UK Geofence
*/
PROGMEM static const int32_t UKgeofence[9 * 2] =
{
  // Visualisation : https://www.dropbox.com/s/c4avx6i7ppigz35/uk_geofence.kml
  // Source        : http://www.ukho.gov.uk/ProductsandServices/Services/Documents/UK%20Territorial%20Sea%20Limits.pdf
  529089020, 2065430,
  509999290, 16040040, 
  507086340, 9448240,
  495252080, -71191410,
  535141850, -52185060,
  543357440, -84704590,
  586026110, -76354980,
  611432350, -5822750,
  551788680, -10986330,
};
/* 
  Netherlands Geofence
*/
PROGMEM static const int32_t Netherlands_geofence[18 * 2] =
{
	507411600,60422900,
	507485500,57239900,
	511516300,58445500,
	514775300,45317000,
	512060100,38138500,
	513701500,33633300,
	523810000,45480000,
	532930000,49620000,
	535540000,68870000,
	532280000,71890000,
	526560000,70380000,
	526610000,67280000,
	524720000,67120000,
	524030000,70220000,
	518960000,67330000,
	518200000,59280000,
	514500000,62020000,
	507411600,60422900,
};


/* 
  Belgium Geofence
*/
PROGMEM static const int32_t Belgium_geofence[25 * 2] =
{
	503115000,37138100,
	503584900,40254500,
	502541000,42087400,
	499806400,41520900,
	499954900,46763900,
	501656200,48082900,
	497971100,48678600,
	495181100,55069800,
	495496100,58244800,
	496623100,59179800,
	498955100,57323800,
	501654400,59632800,
	501420100,61342000,
	503339800,64086700,
	507411600,60422900,
	507485500,57239900,
	511516300,58445500,
	514775300,45317000,
	512060100,38138500,
	513701500,33633300,
	510979700,25490300,
	506962200,28987100,
	507952600,31430100,
	505310100,32899100,
	504554300,36626300,
};
PROGMEM static const int32_t Luxembourg_geofence[11 * 2] =
{
  	495496100,58244800,
	496623100,59179800,
	498955100,57323800,
	501654400,59632800,
	501420100,61342000,
	499754400,61662300,
	498692100,63115100,
	498130300,65192400,
	494673400,63665300,
	495117100,62429500,
	494528400,59776200,
};
PROGMEM static const int32_t Switzerland_geofence[22 * 2] =
{
  	475868900,75735500,
	473948700,71215800,
	464207000,60751200,
	461256100,59605100,
	463717700,67678600,
	459284900,70401300,
	459140000,77950000,
	464560000,84290000,
	458210000,90130000,
	464700000,92810000,
	462450000,10157000,
	466000000,10070000,
	465400000,10444000,
	468520000,10468000,
	470640000,94840000,
	474500000,96570000,
	476640000,91490000,
	477800000,85500000,
	476660000,84120000,
	476440000,86290000,
	475830000,84880000,
	475868900,75735500,
};
PROGMEM static const int32_t Spain_geofence[29 * 2] =
{
  	419290000,32160000,
	410860000,10460000,
	394850000,-03220000,
	387000000,01700000,
	367680000,-21340000,
	367230000,-43980000,
	364000000,-52220000,
	360210000,-55900000,
	364670000,-62390000,
	369390000,-64490000,
	372220000,-71110000,
	371700000,-73920000,
	375500000,-75250000,
	379840000,-72560000,
	380360000,-70000000,
	384630000,-73420000,
	390450000,-69840000,
	396500000,-75320000,
	396660000,-70000000,
	410220000,-69220000,
	415740000,-61980000,
	421500000,-82000000,
	418750000,-88720000,
	430730000,-92020000,
	436770000,-77820000,
	433750000,-17840000,
	426940000,06770000,
	424340000,31660000,
	419290000,32160000,
};
PROGMEM static const int32_t Portugal_geofence[19 * 2] =
{
	371700000,-73920000,
	375500000,-75250000,
	379840000,-72560000,
	380360000,-70000000,
	384630000,-73420000,
	390450000,-69840000,
	396500000,-75320000,
	396660000,-70000000,
	410220000,-69220000,
	415740000,-61980000,
	421500000,-82000000,
	418750000,-88720000,
	411020000,-86270000,
	387340000,-95080000,
	384860000,-88250000,
	370360000,-89320000,
	371260000,-85190000,
	369810000,-79000000,
	371700000,-73920000,
};


/* 
  France Geofence
*/
PROGMEM static const int32_t France_geofence[48 * 2] =
{
	510979700,25490300,
	506962200,28987100,
	507952600,31430100,
	505310100,32899100,
	504554300,36626300,
	503115000,37138100,
	503584900,40254500,
	502541000,42087400,
	499806400,41520900,
	499954900,46763900,
	501656200,48082900,
	497971100,48678600,
	495181100,55069800,
	495496100,58244800,
	494528400,59776200,
	495117100,62429500,
	494673400,63665300,
	491595300,68385800,
	489659400,82307300,
	485722300,77708700,
	480752500,75682600,
	475868900,75735500,
	473948700,71215800,
	464207000,60751200,
	461256100,59605100,
	463717700,67678600,
	459284900,70401300,
	457710000,67740000,
	452620000,71540000,
	451020000,66370000,
	437920000,75260000,
	431110000,62280000,
	435780000,40580000,
	429900000,30410000,
	424340000,31660000,
	426940000,06770000,
	433750000,-17840000,
	462700000,-10590000,
	479920000,-43680000,
	486350000,-44280000,
	486440000,-12930000,
	496470000,-18530000,
	492560000,-1800000,
	494060000,1700000,
	496710000,1580000,
	501270000,15440000,
	508910000,16440000,
	510979700,25490300,
};
/* 
  Germany Geofence
*/
PROGMEM static const int32_t Germany_geofence[76 * 2] =
{
	535540000,68870000,
	532280000,71890000,
	526560000,70380000,
	526610000,67280000,
	524720000,67120000,
	524030000,70220000,
	518960000,67330000,
	518200000,59280000,
	514500000,62020000,
	507411600,60422900,
	503339800,64086700,
	501420100,61342000,
	499754400,61662300,
	498692100,63115100,
	498130300,65192400,
	494673400,63665300,
	491595300,68385800,
	489659400,82307300,
	485722300,77708700,
	480752500,75682600,
	475868900,75735500,
	475868900,75735500,
	476440000,86290000,
	476660000,84120000,
	477800000,85500000,
	476640000,91490000,
	474500000,96570000,
	475913460,97613530,
	472866820,101733400,
	474429500,104534910,
	475765260,104315190,
	474318030,109863280,
	474986480,114367680,
	476154210,122058110,
	477079130,121618650,
	477430170,122607420,
	476931260,124365230,
	476801830,127001950,
	474800880,130187990,
	476875790,130737300,
	477467110,129089360,
	481074310,127551270,
	483124280,133264160,
	485893260,135241700,
	485747900,137548830,
	487779130,138098140,
	493465990,127771000,
	497670740,123925780,
	499229350,125134280,
	503103920,120739750,
	503384490,121893310,
	501944850,123376460,
	504085180,125354000,
	509134240,143701170,
	510413940,142712400,
	510033860,145898440,
	508718450,146337890,
	508302280,147821040,
	508787770,148260500,
	512928410,150238040,
	514813830,149249270,
	515702410,147161870,
	518561390,146008300,
	522412560,147326660,
	522916830,145843510,
	524392690,145458980,
	525696730,146282960,
	528459120,141284180,
	529585660,141558840,
	532750680,144360350,
	539302200,141998290,
	546992340,132934570,
	545338330,111621090,
	548513150,94702150,
	549271420,86132810,
	535540000,68870000,
};