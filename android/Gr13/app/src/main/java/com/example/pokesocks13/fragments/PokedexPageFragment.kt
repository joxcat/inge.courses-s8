package com.example.pokesocks13.fragments

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.databinding.DataBindingUtil
import androidx.fragment.app.Fragment
import com.example.pokesocks13.R
import com.example.pokesocks13.databinding.PokedexPageFragmentBinding
import com.example.pokesocks13.models.Pokemon

class PokedexPageFragment : Fragment() {
    private lateinit var binding: PokedexPageFragmentBinding
    private lateinit var pokemon: Pokemon

    fun setPokemon(pokemon: Pokemon) {
        this.pokemon = pokemon
    }

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        binding =
            DataBindingUtil.inflate(
                inflater,
                R.layout.pokedex_page_fragment,
                container,
                false
            )

        binding.pokemonViewModel = pokemon

        return binding.root
    }
}