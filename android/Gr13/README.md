# PokeSocks - Groupe 13

## About

This application is realised for the purpose of the `Développement Mobile Android` course of 4th year IRC curriculum at [CPE Lyon](https://www.cpe.fr/).

- Made with Kotlin 1.8 and with JVM 1.8
- Minimum Android API SDK version : 26
- Targeted Android API SDK version : 33

### Game rules

1) On first launch, you may choose your starter pokemon between `Bulbizare`, `Salamèche` and `Carapuce`. Once chosen, he will appear in your team.
2) Then you will land on the map page, here you can see the pokeball marker that represents you.
3) Pokemon will appear from *30 meters* up to *5 kilometers* in a circle around you *every 10 second*. A Pokemon leaves after 10 minutes.
4) If you get closer to the Pokemon you can interact with it.
   1) *20 meters* away from it, will be considered as "encountered". Its color will change in the Pokedex and you will be able to see its statistics.
   2) *5 meters* away from it, will be considered as "caught". It will now appear in your team.
   3) Clicking on a Pokemon will show its name in a tooltip
5) In the Pokedex and your team you can click on a Pokemon to show its pokedex page and see its statistics

## Contributors

- Johan PLANCHON [@joxcat](https://github.com/joxcat)
- Julien CAPOSIENA [@julien-cpsn](https://github.com/Julien-cpsn)

## TODO

1) Implement a Rest API call such as [https://overpass-turbo.eu/](https://overpass-turbo.eu/)
2) Notification when caught a pokemon
3) Optimize the redrawing of the map