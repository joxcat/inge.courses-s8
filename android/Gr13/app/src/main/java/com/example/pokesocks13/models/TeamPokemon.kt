package com.example.pokesocks13.models

import androidx.databinding.BaseObservable
import androidx.databinding.Bindable
import com.example.pokesocks13.BR
import com.example.pokesocks13.database.PokemonDao
import kotlinx.serialization.Serializable
import kotlinx.serialization.descriptors.*

@Serializable
class TeamPokemon(val id: Int?, val pokemonId: Int, val position: Int) {
    suspend fun getPokemon(): Pokemon {
        return PokemonDao.getPokemon(pokemonId)!!
    }
}