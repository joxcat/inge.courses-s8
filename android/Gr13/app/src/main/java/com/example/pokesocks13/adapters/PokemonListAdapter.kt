package com.example.pokesocks13.adapters

import android.view.LayoutInflater
import android.view.ViewGroup
import androidx.databinding.DataBindingUtil
import androidx.recyclerview.widget.RecyclerView
import com.example.pokesocks13.R
import com.example.pokesocks13.databinding.PokemonItemBinding
import com.example.pokesocks13.listeners.OnClickOnPokemonItemListener
import com.example.pokesocks13.models.Pokemon


class PokemonListAdapter : RecyclerView.Adapter<PokemonListAdapter.ViewHolder>() {
    var listener: OnClickOnPokemonItemListener? = null
    private lateinit var pokemonList: List<Pokemon>

    fun setPokemonList(pokemonList: List<Pokemon>) {
        this.pokemonList = pokemonList
        notifyDataSetChanged()
    }

    override fun onCreateViewHolder(parent: ViewGroup, viewType: Int): ViewHolder {
        val binding: PokemonItemBinding =
            DataBindingUtil.inflate(
                LayoutInflater.from(parent.context),
                R.layout.pokemon_item,
                parent,
                false
            )
        return ViewHolder(binding)
    }

    override fun onBindViewHolder(holder: ViewHolder, position: Int) {
        val pokemon: Pokemon = pokemonList[position]
        holder.binding.pokemonViewModel = pokemon
    }

    override fun getItemCount(): Int {
        return pokemonList.size
    }

    fun setOnClickOnPokemonItemListener(listener: OnClickOnPokemonItemListener?) {
        this.listener = listener
    }

    inner class ViewHolder(binding: PokemonItemBinding) : RecyclerView.ViewHolder(binding.root) {

        val binding: PokemonItemBinding

        init {
            this.binding = binding
            this.binding.root.setOnClickListener {
                this.binding.pokemonViewModel?.id?.let {
                    listener?.onClickOnPokemonItem(it)
                }
            }
        }
    }
}