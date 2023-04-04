package com.example.pokesocks13

import android.content.Context
import com.example.pokesocks13.models.WildPokemon
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker

fun Marker(context: Context, map: MapView, pokemon: WildPokemon): Marker {
    val image = pokemon.getImage(context, pokemon.image!!)
    val pokemonMarker = Marker(map)

    pokemonMarker.setAnchor(Marker.ANCHOR_CENTER, Marker.ANCHOR_CENTER)
    pokemonMarker.title = pokemon.name
    pokemonMarker.icon = image
    pokemonMarker.image = image
    pokemonMarker.position = pokemon.position.toGeoPoint()
    pokemonMarker.id = pokemon.instanceId

    return pokemonMarker
}