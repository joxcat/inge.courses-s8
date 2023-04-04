package com.example.pokesocks13.models

import android.content.Context
import androidx.databinding.Bindable
import androidx.databinding.ObservableArrayList
import androidx.databinding.ObservableList
import com.example.pokesocks13.Marker
import org.osmdroid.views.MapView
import org.osmdroid.views.overlay.Marker
import java.time.Instant
import java.util.*

class WildPokemon(
    p: Pokemon,
    @get:Bindable var position: Position,
) : Pokemon(p.id, p.name, p.type, p.attack, p.defense, p.weight, p.height, p.discovered, p.image) {
    val instanceId: String = p.id.toString() + UUID.randomUUID().toString()
    val spawnedTime: Instant = Instant.now()
}

class WildPokemonListener(
    private val map: MapView,
    private val context: Context,
    private val wildPokemonsCache: ArrayList<WildPokemon>
) : ObservableList.OnListChangedCallback<ObservableArrayList<WildPokemon>>() {

    override fun onItemRangeInserted(
        sender: ObservableArrayList<WildPokemon>?,
        positionStart: Int,
        itemCount: Int
    ) {
        var needRedraw = false

        for (pokemon in sender?.subList(positionStart, positionStart + itemCount).orEmpty()) {
            map.overlays.add(Marker(context, map, pokemon))
            wildPokemonsCache.add(pokemon)

            needRedraw = true
        }

        // TODO: Specify bounding box
        if (needRedraw) map.postInvalidate()
    }

    override fun onItemRangeRemoved(
        sender: ObservableArrayList<WildPokemon>?,
        positionStart: Int,
        itemCount: Int
    ) {
        var needRedraw = false

        val iterator = wildPokemonsCache.subList(positionStart, positionStart + itemCount).iterator()
        while (iterator.hasNext()) {
            val pokemon = iterator.next()
            map.overlays.removeIf { (it as Marker).id == pokemon.instanceId }
            iterator.remove()

            needRedraw = true
        }

        // TODO: Specify bounding box
        if (needRedraw) map.postInvalidate()
    }

    override fun onChanged(sender: ObservableArrayList<WildPokemon>?) {
        /* Do nothing */
    }

    override fun onItemRangeChanged(
        sender: ObservableArrayList<WildPokemon>?,
        positionStart: Int,
        itemCount: Int
    ) {
        for (pokemon in sender?.subList(positionStart, positionStart + itemCount).orEmpty()) {
            println("Range changed ${pokemon.name}")
        }
    }

    override fun onItemRangeMoved(
        sender: ObservableArrayList<WildPokemon>?,
        fromPosition: Int,
        toPosition: Int,
        itemCount: Int
    ) {
        for (pokemon in sender?.subList(toPosition, toPosition + itemCount).orEmpty()) {
            println("Moved ${pokemon.name}")
        }
    }
}