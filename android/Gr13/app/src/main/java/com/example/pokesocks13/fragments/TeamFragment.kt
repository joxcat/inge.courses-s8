package com.example.pokesocks13.fragments

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.databinding.DataBindingUtil
import androidx.fragment.app.Fragment
import androidx.recyclerview.widget.LinearLayoutManager
import com.example.pokesocks13.adapters.PokemonListAdapter
import com.example.pokesocks13.R
import com.example.pokesocks13.database.PokemonDao
import com.example.pokesocks13.databinding.PokedexFragmentBinding
import com.example.pokesocks13.listeners.OnClickOnPokemonItemListener
import com.example.pokesocks13.models.Pokemon
import com.example.pokesocks13.models.TeamPokemon
import kotlinx.coroutines.runBlocking


class TeamFragment : Fragment() {
    private var listener: OnClickOnPokemonItemListener? = null
    private lateinit var pokemonList: List<Pokemon>
    private lateinit var adapter: PokemonListAdapter

    fun setPokemonList(team: List<TeamPokemon>) {
        if (::adapter.isInitialized) {
            adapter.setPokemonList(pokemonList)
        }

        val tempPokemonList = mutableListOf<Pokemon>()

        runBlocking {
            for (teamPokemon in team.sortedBy { it.position }) {
                tempPokemonList.add(PokemonDao.getPokemon(teamPokemon.pokemonId)!!)
            }

            pokemonList = tempPokemonList
        }
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        val binding: PokedexFragmentBinding =
            DataBindingUtil.inflate(
                inflater,
                R.layout.pokedex_fragment,
                container,
                false
            )
        binding.pokemonList.layoutManager = LinearLayoutManager(binding.root.context)

        adapter = PokemonListAdapter()
        adapter.setPokemonList(pokemonList)
        adapter.setOnClickOnPokemonItemListener(this.listener)
        binding.pokemonList.adapter = adapter

        return binding.root
    }

    fun setOnClickOnPokemonItemListener(listener: OnClickOnPokemonItemListener?) {
        this.listener = listener
    }

}