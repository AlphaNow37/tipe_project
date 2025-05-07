macro_rules! make_trait_alias {
    (
        $new: ident = [$($old: tt)*] { $($content: tt)* }
    ) => {
        pub trait $new: $($old)* { $($content)* }
        impl<T: $($old)*> $new for T {}
    };
}
pub(crate) use make_trait_alias;