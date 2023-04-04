package com.example.pokesocks13.models

import android.content.Context
import android.graphics.Color
import android.graphics.drawable.Drawable
import androidx.core.content.res.ResourcesCompat
import androidx.databinding.BaseObservable
import androidx.databinding.Bindable
import com.example.pokesocks13.BR
import com.example.pokesocks13.R
import com.example.pokesocks13.serializers.PokemonSerializer
import kotlinx.serialization.Serializable

@Serializable(with = PokemonSerializer::class)
open class Pokemon(
    id: Int?,
    name: String,
    type: PokemonType,
    attack: Int,
    defense: Int,
    weight: Float,
    height: Float,
    discovered: Boolean,
    image: Int?
) : BaseObservable() {

    // ID
    @get:Bindable
    var id: Int? = id
        set(value) {
            field = value
            notifyPropertyChanged(BR.id)
        }

    @Bindable
    fun getIdAsString(): String {
        return "#$id"
    }

    // NAME
    @get:Bindable
    var name: String = name
        set(value) {
            field = value
            notifyPropertyChanged(BR.name)
        }

    // TYPE
    @get:Bindable
    var type: PokemonType = type
        set(value) {
            field = value
            notifyPropertyChanged(BR.type)
        }

    fun getFrenchType(): String {
        return typeToFrenchString(type)
    }

    fun getTypeImage(context: Context): Drawable? {
        val res: Int = when (type) {
            PokemonType.Steel -> R.drawable.acier
            PokemonType.Fight -> R.drawable.combat
            PokemonType.Dragon -> R.drawable.dragon
            PokemonType.Water -> R.drawable.eau
            PokemonType.Electric -> R.drawable.electrique
            PokemonType.Fairy -> R.drawable.fee
            PokemonType.Fire -> R.drawable.feu
            PokemonType.Ice -> R.drawable.glace
            PokemonType.Insect -> R.drawable.insecte
            PokemonType.Normal -> R.drawable.normal
            PokemonType.Plant -> R.drawable.plante
            PokemonType.Poison -> R.drawable.poison
            PokemonType.Psy -> R.drawable.psy
            PokemonType.Rock -> R.drawable.roche
            PokemonType.Ground -> R.drawable.sol
            PokemonType.Spectre -> R.drawable.spectre
            PokemonType.Darkness -> R.drawable.tenebre
            PokemonType.Flight -> R.drawable.vol
            PokemonType.Sock -> R.drawable.normal
        }

        return ResourcesCompat.getDrawable(context.resources, res, context.theme)
    }

    // ATTACK
    @get:Bindable
    var attack: Int = attack
        set(value) {
            field = value
            notifyPropertyChanged(BR.attack)
        }

    @Bindable
    fun getAttackAsString(): String {
        return if (discovered) {
            "Attaque: $attack"
        } else {
            "Attaque: N\\A"
        }
    }

    // DEFENSE
    @get:Bindable
    var defense: Int = defense
        set(value) {
            field = value
            notifyPropertyChanged(BR.defense)
        }

    @Bindable
    fun getDefenseAsString(): String {
        return if (discovered) {
            "Défense: $defense"
        } else {
            "Défense: N\\A"
        }
    }

    // WEIGHT
    @get:Bindable
    var weight: Float = weight
        set(value) {
            field = value
            notifyPropertyChanged(BR.weight)
        }

    @Bindable
    fun getWeightAsString(): String {
        return if (discovered) {
            "Weight: ${weight}Kg"
        } else {
            "Weight: N\\A"
        }
    }

    // HEIGHT
    @get:Bindable
    var height: Float = height
        set(value) {
            field = value
            notifyPropertyChanged(BR.height)
        }

    @Bindable
    fun getHeightAsString(): String {
        return if (discovered) {
            "Height: ${height}m"
        } else {
            "Height: N\\A"
        }
    }

    // DISCOVERED
    @get:Bindable
    var discovered: Boolean = discovered
        set(value) {
            field = value
            notifyPropertyChanged(BR.discovered)
        }

    fun getColorIfDiscovered(): Int {
        return if (discovered) {
            Color.rgb(0, 0, 150)
        } else {
            Color.rgb(0, 0, 0)
        }
    }

    // IMAGE
    @get:Bindable
    var image: Int? = image
        set(value) {
            field = value
            notifyPropertyChanged(BR.image)
        }

    fun getImage(context: Context, res: Int): Drawable? {
        return if (res != -1)
            ResourcesCompat.getDrawable(context.resources, res, context.theme)
        else
            null
    }
}