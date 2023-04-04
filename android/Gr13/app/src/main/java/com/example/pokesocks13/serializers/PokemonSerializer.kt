package com.example.pokesocks13.serializers

import com.example.pokesocks13.models.Pokemon
import com.example.pokesocks13.models.PokemonType
import com.example.pokesocks13.models.frenchStringToType
import kotlinx.serialization.KSerializer
import kotlinx.serialization.descriptors.SerialDescriptor
import kotlinx.serialization.descriptors.buildClassSerialDescriptor
import kotlinx.serialization.descriptors.element
import kotlinx.serialization.encoding.CompositeDecoder
import kotlinx.serialization.encoding.Decoder
import kotlinx.serialization.encoding.Encoder
import kotlinx.serialization.encoding.decodeStructure
import kotlin.random.Random

object PokemonSerializer : KSerializer<Pokemon> {
    override val descriptor: SerialDescriptor = buildClassSerialDescriptor("Pokemon") {
        element<Int>("id", isOptional = true)
        element<String>("name")
        element<PokemonType>("type")
        element<Int>("attack")
        element<Int>("defense")
        element<Float>("weight")
        element<Float>("height")
        element<Boolean>("discovered")
        element<Int>("image", isOptional = true)
    }

    override fun serialize(encoder: Encoder, value: Pokemon) {
        encoder.encodeInt(value.id!!)
        encoder.encodeString(value.name)
        encoder.encodeString(value.type.toString())
        encoder.encodeInt(value.attack)
        encoder.encodeInt(value.defense)
        encoder.encodeFloat(value.weight)
        encoder.encodeFloat(value.height)
        encoder.encodeBoolean(value.discovered)
        encoder.encodeString("p${value.image}")
    }

    override fun deserialize(decoder: Decoder): Pokemon {
        var id: Int? = null
        var name = ""
        var image = 0
        var type: PokemonType = PokemonType.Sock
        var attack = 0
        var defense = 0
        var weight: Float = 0.0.toFloat()
        var height: Float = 0.0.toFloat()
        decoder.decodeStructure(descriptor) {
            while (true) {
                when (val index = decodeElementIndex(descriptor)) {
                    0 -> id = decodeIntElement(descriptor, 0)
                    1 -> name = decodeStringElement(descriptor, 1)
                    2 -> type = frenchStringToType(decodeStringElement(descriptor, 2))
                    3 -> attack = decodeIntElement(descriptor, 3) * Random.nextInt(0, 30)
                    4 -> defense = decodeIntElement(descriptor, 4) * Random.nextInt(0, 30)
                    5 -> weight = decodeFloatElement(descriptor, 5)
                    6 -> height = decodeFloatElement(descriptor, 6)
                    8 -> {
                        image = decodeStringElement(descriptor, 8).replace("p", "").toInt()
                    }
                    CompositeDecoder.DECODE_DONE -> break
                    else -> error("Unexpected index: $index")
                }
            }
        }

        return Pokemon(id, name, type, attack, defense, weight, height, false, image)
    }
}